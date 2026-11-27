#include "mavlink2_tx.h"

#include <math.h>
#include <string.h>

static void crc_accumulate(uint8_t byte, uint16_t *crc) {
    uint8_t tmp = (uint8_t)(byte ^ (uint8_t)(*crc & 0xFFU));
    tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4U));
    *crc = (uint16_t)((*crc >> 8U) ^ ((uint16_t)tmp << 8U) ^ ((uint16_t)tmp << 3U) ^ ((uint16_t)tmp >> 4U));
}

uint16_t openfc_mavlink_crc16_mcrf4xx(const uint8_t *data, size_t len, uint8_t crc_extra) {
    uint16_t crc = 0xFFFFU;
    if (data != NULL) {
        for (size_t i = 0U; i < len; ++i) {
            crc_accumulate(data[i], &crc);
        }
    }
    crc_accumulate(crc_extra, &crc);
    return crc;
}

static void put_u32_le(uint8_t *dst, uint32_t v) {
    dst[0] = (uint8_t)(v & 0xFFU);
    dst[1] = (uint8_t)((v >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((v >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((v >> 24U) & 0xFFU);
}

static void put_u64_le(uint8_t *dst, uint64_t v) {
    put_u32_le(dst, (uint32_t)(v & 0xFFFFFFFFU));
    put_u32_le(dst + 4U, (uint32_t)((v >> 32U) & 0xFFFFFFFFU));
}

static void put_f32_le(uint8_t *dst, float f) {
    uint32_t bits = 0U;
    memcpy(&bits, &f, sizeof(bits));
    put_u32_le(dst, bits);
}

static size_t trim_v2_payload(const uint8_t *payload, size_t len) {
    while (len > 1U && payload[len - 1U] == 0U) {
        len--;
    }
    return len;
}

static size_t finish_v2_frame(
    openfc_mavlink2_tx_t *tx,
    uint32_t msg_id,
    uint8_t crc_extra,
    const uint8_t *payload,
    size_t payload_len,
    uint8_t *out_buf,
    size_t out_cap) {
    if (tx == NULL || payload == NULL || payload_len == 0U || payload_len > 255U || msg_id > 0xFFFFFFU) {
        return 0U;
    }
    size_t trimmed_len = trim_v2_payload(payload, payload_len);
    size_t total_len = 10U + trimmed_len + 2U;
    if (out_buf == NULL || out_cap < total_len) {
        return 0U;
    }

    out_buf[0] = OPENFC_MAVLINK2_STX;
    out_buf[1] = (uint8_t)trimmed_len;
    out_buf[2] = 0U; /* incompat_flags */
    out_buf[3] = 0U; /* compat_flags */
    out_buf[4] = tx->seq++;
    out_buf[5] = tx->sys_id;
    out_buf[6] = tx->comp_id;
    out_buf[7] = (uint8_t)(msg_id & 0xFFU);
    out_buf[8] = (uint8_t)((msg_id >> 8U) & 0xFFU);
    out_buf[9] = (uint8_t)((msg_id >> 16U) & 0xFFU);

    memcpy(out_buf + 10U, payload, trimmed_len);

    uint16_t crc = openfc_mavlink_crc16_mcrf4xx(out_buf + 1U, 9U + trimmed_len, crc_extra);
    out_buf[10U + trimmed_len] = (uint8_t)(crc & 0xFFU);
    out_buf[11U + trimmed_len] = (uint8_t)((crc >> 8U) & 0xFFU);
    return total_len;
}

void openfc_mavlink2_tx_init(openfc_mavlink2_tx_t *tx, uint8_t sys_id, uint8_t comp_id) {
    if (tx == NULL) {
        return;
    }
    tx->sys_id = sys_id;
    tx->comp_id = comp_id;
    tx->seq = 0U;
}

size_t openfc_mavlink2_pack_heartbeat(
    openfc_mavlink2_tx_t *tx,
    uint8_t system_status,
    uint8_t *out_buf,
    size_t out_cap) {
    if (tx == NULL) {
        return 0U;
    }
    uint8_t payload[9];
    put_u32_le(payload + 0U, 0U);                         /* custom_mode */
    payload[4] = OPENFC_MAV_TYPE_ONBOARD_CONTROLLER;      /* type */
    payload[5] = OPENFC_MAV_AUTOPILOT_INVALID;            /* autopilot */
    payload[6] = 0U;                                      /* base_mode */
    payload[7] = system_status;                           /* system_status */
    payload[8] = 3U;                                      /* mavlink_version */

    return finish_v2_frame(tx, OPENFC_MAVLINK_MSG_ID_HEARTBEAT, OPENFC_MAVLINK_CRC_EXTRA_HEARTBEAT,
                           payload, sizeof(payload), out_buf, out_cap);
}

size_t openfc_mavlink2_pack_landing_target(
    openfc_mavlink2_tx_t *tx,
    const openfc_lt_measurement_t *meas,
    uint8_t *out_buf,
    size_t out_cap) {
    if (tx == NULL || meas == NULL) {
        return 0U;
    }
    /* Trust-boundary guard: never emit NaN/Inf or unsupported position_valid!=0 with zero x/y/z */
    if (!isfinite(meas->angle_x_rad) || !isfinite(meas->angle_y_rad) ||
        !isfinite(meas->distance_m) || !isfinite(meas->size_x_rad) || !isfinite(meas->size_y_rad) ||
        meas->distance_m < 0.0f || meas->size_x_rad < 0.0f || meas->size_y_rad < 0.0f ||
        meas->position_valid != 0U) {
        return 0U;
    }
    /* Wire order (<QfffffBBfff4fBB) verified against pymavlink v20 ardupilotmega (60 bytes) */
    uint8_t payload[60];
    memset(payload, 0, sizeof(payload));

    put_u64_le(payload + 0U, meas->time_usec);
    put_f32_le(payload + 8U, meas->angle_x_rad);
    put_f32_le(payload + 12U, meas->angle_y_rad);
    put_f32_le(payload + 16U, meas->distance_m);
    put_f32_le(payload + 20U, meas->size_x_rad);
    put_f32_le(payload + 24U, meas->size_y_rad);
    payload[28] = 0U;                                        /* target_num */
    payload[29] = OPENFC_MAV_FRAME_BODY_FRD;                 /* frame = 12 */
    /* x, y, z remain 0.0f while position_valid == 0 */
    put_f32_le(payload + 42U, 1.0f);                         /* q = [1, 0, 0, 0] */
    payload[58] = OPENFC_LANDING_TARGET_TYPE_VISION_FIDUCIAL; /* type = 2 */
    payload[59] = meas->position_valid;

    return finish_v2_frame(tx, OPENFC_MAVLINK_MSG_ID_LANDING_TARGET,
                           OPENFC_MAVLINK_CRC_EXTRA_LANDING_TARGET,
                           payload, sizeof(payload), out_buf, out_cap);
}
