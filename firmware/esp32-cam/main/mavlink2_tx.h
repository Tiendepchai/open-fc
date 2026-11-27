#ifndef OPENFC_MAVLINK2_TX_H
#define OPENFC_MAVLINK2_TX_H

#include <stddef.h>
#include <stdint.h>

#include "landing_target_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPENFC_MAVLINK2_STX 0xFDU
#define OPENFC_MAVLINK_MSG_ID_HEARTBEAT 0U
#define OPENFC_MAVLINK_CRC_EXTRA_HEARTBEAT 50U
#define OPENFC_MAVLINK_MSG_ID_LANDING_TARGET 149U
#define OPENFC_MAVLINK_CRC_EXTRA_LANDING_TARGET 200U

#define OPENFC_MAV_TYPE_ONBOARD_CONTROLLER 18U
#define OPENFC_MAV_AUTOPILOT_INVALID 8U
#define OPENFC_MAV_STATE_STANDBY 3U
#define OPENFC_MAV_STATE_ACTIVE 4U
#define OPENFC_MAV_FRAME_BODY_FRD 12U
#define OPENFC_LANDING_TARGET_TYPE_VISION_FIDUCIAL 2U

#define OPENFC_MAVLINK2_MAX_TX_BUF 80U

typedef struct {
    uint8_t sys_id;
    uint8_t comp_id;
    uint8_t seq;
} openfc_mavlink2_tx_t;

void openfc_mavlink2_tx_init(openfc_mavlink2_tx_t *tx, uint8_t sys_id, uint8_t comp_id);

uint16_t openfc_mavlink_crc16_mcrf4xx(const uint8_t *data, size_t len, uint8_t crc_extra);

/**
 * Pack a MAVLink2 HEARTBEAT (#0) into out_buf (requires >= 21 bytes).
 * Returns total encoded packet length in bytes, or 0 on invalid buffer.
 */
size_t openfc_mavlink2_pack_heartbeat(
    openfc_mavlink2_tx_t *tx,
    uint8_t system_status,
    uint8_t *out_buf,
    size_t out_cap);

/**
 * Pack a MAVLink2 LANDING_TARGET (#149, MAV_FRAME_BODY_FRD, VISION_FIDUCIAL) into out_buf
 * (requires >= 71 bytes for position_valid==0; OPENFC_MAVLINK2_MAX_TX_BUF recommended).
 * Applies MAVLink2 trailing-zero payload truncation and rejects non-finite fields,
 * negative distance/size, or unsupported position_valid != 0.
 * Returns total encoded packet length in bytes, or 0 on invalid input/buffer.
 */
size_t openfc_mavlink2_pack_landing_target(
    openfc_mavlink2_tx_t *tx,
    const openfc_lt_measurement_t *meas,
    uint8_t *out_buf,
    size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif /* OPENFC_MAVLINK2_TX_H */
