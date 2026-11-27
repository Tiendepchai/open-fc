#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../main/landing_target_math.h"
#include "../main/marker_detector.h"
#include "../main/mavlink2_tx.h"

#define TEST_PI_F 3.14159265358979323846f

static void test_mavlink2_golden_vectors_and_guards(void) {
    /* Verified byte-for-byte against pymavlink 2.4.50 v20 ardupilotmega */
    const uint8_t expected_hb[21] = {
        0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0xBF, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x12, 0x08, 0x00, 0x04, 0x03, 0xAE, 0xC6
    };
    const uint8_t expected_lt[71] = {
        0xFD, 0x3B, 0x00, 0x00, 0x00, 0x01, 0xBF, 0x95, 0x00, 0x00,
        0x4E, 0x61, 0xBC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCC,
        0x4C, 0x3D, 0x8F, 0xC2, 0xF5, 0xBC, 0x00, 0x00, 0x20, 0x40,
        0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x0C,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xD7, 0x27
    };
    /* Sequence 255 -> wraps to 0 on next packet (pymavlink 2.4.50 verified) */
    const uint8_t expected_lt_seq255[71] = {
        0xFD, 0x3B, 0x00, 0x00, 0xFF, 0x01, 0xBF, 0x95, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x3E, 0xB9
    };

    uint8_t buf[OPENFC_MAVLINK2_MAX_TX_BUF];
    openfc_mavlink2_tx_t tx;

    openfc_mavlink2_tx_init(NULL, 1U, 191U);
    openfc_mavlink2_tx_init(&tx, 1U, 191U);

    /* Short buffer MUST fail without advancing tx.seq */
    assert(openfc_mavlink2_pack_heartbeat(&tx, OPENFC_MAV_STATE_ACTIVE, buf, 20U) == 0U);
    assert(tx.seq == 0U);

    size_t hb_len = openfc_mavlink2_pack_heartbeat(&tx, OPENFC_MAV_STATE_ACTIVE, buf, sizeof(buf));
    assert(hb_len == sizeof(expected_hb));
    assert(memcmp(buf, expected_hb, sizeof(expected_hb)) == 0);
    assert(tx.seq == 1U);

    /* STANDBY heartbeat packs cleanly */
    size_t hb_stby = openfc_mavlink2_pack_heartbeat(&tx, OPENFC_MAV_STATE_STANDBY, buf, sizeof(buf));
    assert(hb_stby == sizeof(expected_hb));
    assert(buf[17] == OPENFC_MAV_STATE_STANDBY);

    /* Exact LANDING_TARGET golden vector */
    openfc_mavlink2_tx_init(&tx, 1U, 191U);
    openfc_lt_measurement_t m;
    memset(&m, 0, sizeof(m));
    m.time_usec = 12345678ULL;
    m.angle_x_rad = 0.05f;
    m.angle_y_rad = -0.03f;
    m.distance_m = 2.5f;
    m.size_x_rad = 0.1f;
    m.size_y_rad = 0.1f;
    m.position_valid = 0U;

    /* Short buffer (70 < 71) MUST fail without advancing seq */
    assert(openfc_mavlink2_pack_landing_target(&tx, &m, buf, 70U) == 0U);
    assert(tx.seq == 0U);

    size_t lt_len = openfc_mavlink2_pack_landing_target(&tx, &m, buf, sizeof(buf));
    assert(lt_len == sizeof(expected_lt));
    assert(memcmp(buf, expected_lt, sizeof(expected_lt)) == 0);
    assert(tx.seq == 1U);

    /* Sequence rollover 255 -> 0 matches pymavlink byte-for-byte */
    tx.seq = 255U;
    memset(&m, 0, sizeof(m));
    size_t lt255_len = openfc_mavlink2_pack_landing_target(&tx, &m, buf, sizeof(buf));
    assert(lt255_len == sizeof(expected_lt_seq255));
    assert(memcmp(buf, expected_lt_seq255, sizeof(expected_lt_seq255)) == 0);
    assert(tx.seq == 0U);

    /* Wire trust-boundary guards: NaN, Inf, negative size/distance, unsupported position_valid!=0 */
    openfc_lt_measurement_t bad = m;
    bad.angle_x_rad = NAN;
    assert(openfc_mavlink2_pack_landing_target(&tx, &bad, buf, sizeof(buf)) == 0U);

    bad = m;
    bad.angle_y_rad = INFINITY;
    assert(openfc_mavlink2_pack_landing_target(&tx, &bad, buf, sizeof(buf)) == 0U);

    bad = m;
    bad.distance_m = -0.1f;
    assert(openfc_mavlink2_pack_landing_target(&tx, &bad, buf, sizeof(buf)) == 0U);

    bad = m;
    bad.size_x_rad = -0.01f;
    assert(openfc_mavlink2_pack_landing_target(&tx, &bad, buf, sizeof(buf)) == 0U);

    bad = m;
    bad.position_valid = 1U;
    assert(openfc_mavlink2_pack_landing_target(&tx, &bad, buf, sizeof(buf)) == 0U);

    assert(openfc_mavlink2_pack_landing_target(NULL, &m, buf, sizeof(buf)) == 0U);
    assert(openfc_mavlink2_pack_landing_target(&tx, NULL, buf, sizeof(buf)) == 0U);
    assert(openfc_mavlink2_pack_landing_target(&tx, &m, NULL, sizeof(buf)) == 0U);
    assert(tx.seq == 0U);
}

static void assert_ardupilot_los_parity(
    const openfc_cam_calib_t *calib,
    float u_px,
    float v_px,
    float expect_body_fwd,
    float expect_body_right) {
    openfc_lt_measurement_t out;
    openfc_lt_status_t st = openfc_compute_landing_target(
        calib, NULL, true, u_px, v_px, 24.0f, 24.0f, 0.8f, 1000000ULL, 1010000ULL, &out);
    assert(st == OPENFC_LT_OK);

    /* Exact ArduCopter AC_PrecLand_Companion.cpp:45 reconstruction */
    float los_x = -tanf(out.angle_y_rad);
    float los_y = tanf(out.angle_x_rad);
    float los_z = 1.0f;
    float los_norm = sqrtf((los_x * los_x) + (los_y * los_y) + (los_z * los_z));

    float exp_norm = sqrtf((expect_body_fwd * expect_body_fwd) +
                           (expect_body_right * expect_body_right) + 1.0f);

    assert(fabsf((los_x / los_norm) - (expect_body_fwd / exp_norm)) < 1e-5f);
    assert(fabsf((los_y / los_norm) - (expect_body_right / exp_norm)) < 1e-5f);
    assert(fabsf((los_z / los_norm) - (1.0f / exp_norm)) < 1e-5f);
}

static void test_yaw_rotations_against_ardupilot(void) {
    openfc_cam_calib_t calib = {
        .img_width = 320U,
        .img_height = 240U,
        .fx = 250.0f,
        .fy = 250.0f,
        .cx = 160.0f,
        .cy = 120.0f,
        .cam_yaw_offset_rad = 0.0f,
        .min_confidence = 0.2f,
        .max_frame_age_us = 150000ULL,
        .max_step_rad = 0.35f,
        .outlier_window_us = 300000ULL,
    };

    /* Test pixel (u=210, v=70): cam_right = +0.20, cam_fwd = +0.20 (top-right in image) */
    /* Also asymmetric pixel (u=235, v=95): cam_right = +0.30, cam_fwd = +0.10 */
    const float u_asym = 235.0f;
    const float v_asym = 95.0f;
    const float cr = 0.30f;
    const float cf = 0.10f;

    /* 0 deg: body_fwd = +cf, body_right = +cr */
    calib.cam_yaw_offset_rad = 0.0f;
    assert_ardupilot_los_parity(&calib, 160.0f, 120.0f, 0.0f, 0.0f);
    assert_ardupilot_los_parity(&calib, u_asym, v_asym, cf, cr);

    /* +90 deg CW about +Z Down: camera top (+cf) -> +Y Right; camera right (+cr) -> -X Back */
    calib.cam_yaw_offset_rad = 0.5f * TEST_PI_F;
    assert_ardupilot_los_parity(&calib, u_asym, v_asym, -cr, cf);

    /* +180 deg: camera top (+cf) -> -X Back; camera right (+cr) -> -Y Left */
    calib.cam_yaw_offset_rad = TEST_PI_F;
    assert_ardupilot_los_parity(&calib, u_asym, v_asym, -cf, -cr);

    /* +270 deg (-90 deg): camera top (+cf) -> -Y Left; camera right (+cr) -> +X Forward */
    calib.cam_yaw_offset_rad = 1.5f * TEST_PI_F;
    assert_ardupilot_los_parity(&calib, u_asym, v_asym, cr, -cf);
}

static void test_rejection_gates_wrap_and_occlusion_recovery(void) {
    openfc_cam_calib_t calib = {
        .img_width = 320U,
        .img_height = 240U,
        .fx = 250.0f,
        .fy = 250.0f,
        .cx = 160.0f,
        .cy = 120.0f,
        .cam_yaw_offset_rad = 0.0f,
        .min_confidence = 0.2f,
        .max_frame_age_us = 150000ULL,
        .max_step_rad = 0.35f,
        .outlier_window_us = 300000ULL,
    };
    openfc_lt_filter_state_t st;
    openfc_lt_filter_reset(&st);
    openfc_lt_measurement_t out;

    /* 1. Uncalibrated intrinsics & invalid config guards */
    openfc_cam_calib_t bad_cal = calib;
    bad_cal.fx = 0.0f;
    assert(openfc_compute_landing_target(&bad_cal, &st, true, 160.0f, 120.0f, 30.0f, 30.0f, 0.8f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_UNCALIBRATED);
    bad_cal = calib;
    bad_cal.fy = 0.0f;
    assert(openfc_compute_landing_target(&bad_cal, &st, true, 160.0f, 120.0f, 30.0f, 30.0f, 0.8f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_UNCALIBRATED);
    bad_cal = calib;
    bad_cal.min_confidence = NAN;
    assert(openfc_compute_landing_target(&bad_cal, &st, true, 160.0f, 120.0f, 30.0f, 30.0f, 0.01f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_UNCALIBRATED);
    bad_cal = calib;
    bad_cal.max_step_rad = NAN;
    assert(openfc_compute_landing_target(&bad_cal, &st, true, 160.0f, 120.0f, 30.0f, 30.0f, 0.8f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_UNCALIBRATED);
    bad_cal = calib;
    bad_cal.fx = 1e-40f; /* overflows cam_right to +Inf -> must be caught before atan2f */
    assert(openfc_compute_landing_target(&bad_cal, &st, true, 210.0f, 70.0f, 30.0f, 30.0f, 0.8f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_NON_FINITE);

    /* 2. Input bounds, NaN, low confidence, not detected */
    assert(openfc_compute_landing_target(&calib, &st, false, 160.0f, 120.0f, 30.0f, 30.0f, 0.8f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_NOT_DETECTED);
    assert(openfc_compute_landing_target(&calib, &st, true, NAN, 120.0f, 30.0f, 30.0f, 0.8f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_NON_FINITE);
    assert(openfc_compute_landing_target(&calib, &st, true, 320.0f, 120.0f, 30.0f, 30.0f, 0.8f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_OUT_OF_BOUNDS);
    assert(openfc_compute_landing_target(&calib, &st, true, -0.1f, 120.0f, 30.0f, 30.0f, 0.8f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_OUT_OF_BOUNDS);
    assert(openfc_compute_landing_target(&calib, &st, true, 160.0f, 120.0f, 30.0f, 30.0f, 0.1f,
                                         1000000ULL, 1010000ULL, &out) == OPENFC_LT_ERR_LOW_CONFIDENCE);

    /* 3. Target top-right of image (u > cx => +Y Right, v < cy => +X Forward) */
    assert(openfc_compute_landing_target(&calib, &st, true, 210.0f, 70.0f, 30.0f, 30.0f, 0.8f,
                                         1000000ULL, 1020000ULL, &out) == OPENFC_LT_OK);
    assert(out.angle_x_rad > 0.19f && out.angle_x_rad < 0.20f);
    assert(out.angle_y_rad < -0.19f && out.angle_y_rad > -0.20f);

    /* 4. Stale & inverted timestamps */
    assert(openfc_compute_landing_target(&calib, &st, true, 210.0f, 70.0f, 30.0f, 30.0f, 0.8f,
                                         1050000ULL, 1200001ULL, &out) == OPENFC_LT_ERR_STALE_FRAME);
    assert(openfc_compute_landing_target(&calib, &st, true, 210.0f, 70.0f, 30.0f, 30.0f, 0.8f,
                                         1050000ULL, 1049999ULL, &out) == OPENFC_LT_ERR_STALE_FRAME);
    /* Out-of-order older capture_us (< prev_capture_us) must be rejected without rewinding filter */
    assert(openfc_compute_landing_target(&calib, &st, true, 210.0f, 70.0f, 30.0f, 30.0f, 0.8f,
                                         950000ULL, 960000ULL, &out) == OPENFC_LT_ERR_STALE_FRAME);
    assert(st.prev_capture_us == 1000000ULL);

    /* 5. Spurious single-frame outlier rejected within outlier_window_us */
    assert(openfc_compute_landing_target(&calib, &st, true, 10.0f, 230.0f, 30.0f, 30.0f, 0.8f,
                                         1050000ULL, 1060000ULL, &out) == OPENFC_LT_ERR_OUTLIER_JUMP);
    /* Returning to real target position still passes */
    assert(openfc_compute_landing_target(&calib, &st, true, 208.0f, 72.0f, 30.0f, 30.0f, 0.8f,
                                         1100000ULL, 1110000ULL, &out) == OPENFC_LT_OK);

    /* 6. Sustained occlusion (> outlier_window_us) MUST allow immediate re-acquisition at new pose */
    for (uint64_t t = 1150000ULL; t <= 1400000ULL; t += 50000ULL) {
        assert(openfc_compute_landing_target(&calib, &st, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                                             t, t + 10000ULL, &out) == OPENFC_LT_ERR_NOT_DETECTED);
    }
    /* dt = 1450000 - 1100000 = 350000 > 300000 => re-acquired, NOT permanently rejected! */
    assert(openfc_compute_landing_target(&calib, &st, true, 25.0f, 215.0f, 30.0f, 30.0f, 0.85f,
                                         1450000ULL, 1465000ULL, &out) == OPENFC_LT_OK);
    assert(st.prev_capture_us == 1450000ULL);

    /* 7. Brief occlusion (< outlier_window_us) followed by steady new target re-acquires once window elapses */
    assert(openfc_compute_landing_target(&calib, &st, true, 260.0f, 40.0f, 30.0f, 30.0f, 0.85f,
                                         1600000ULL, 1610000ULL, &out) == OPENFC_LT_ERR_OUTLIER_JUMP);
    assert(openfc_compute_landing_target(&calib, &st, true, 260.0f, 40.0f, 30.0f, 30.0f, 0.85f,
                                         1750000ULL, 1760000ULL, &out) == OPENFC_LT_ERR_OUTLIER_JUMP);
    /* dt = 1750001 - 1450000 = 300001 > 300000 => re-acquires cleanly */
    assert(openfc_compute_landing_target(&calib, &st, true, 260.0f, 40.0f, 30.0f, 30.0f, 0.85f,
                                         1750001ULL, 1760001ULL, &out) == OPENFC_LT_OK);
    assert(st.prev_capture_us == 1750001ULL);

    /* 8. uint64_t timestamp wraparound safety (age check + outlier filter + post-occlusion recovery) */
    openfc_lt_filter_reset(&st);
    const uint64_t near_max = UINT64_MAX - 10000ULL;
    /* capture near UINT64_MAX, now wrapped past 0 (age = 20001 us <= 150000 us) */
    assert(openfc_compute_landing_target(&calib, &st, true, 210.0f, 70.0f, 30.0f, 30.0f, 0.85f,
                                         near_max, 10000ULL, &out) == OPENFC_LT_OK);
    /* Wrapped stale frame (age = 210001 us > 150000 us) rejected */
    assert(openfc_compute_landing_target(&calib, &st, true, 210.0f, 70.0f, 30.0f, 30.0f, 0.85f,
                                         near_max, 200000ULL, &out) == OPENFC_LT_ERR_STALE_FRAME);
    /* Outlier jump across wrap within 50 ms rejected */
    assert(openfc_compute_landing_target(&calib, &st, true, 20.0f, 220.0f, 30.0f, 30.0f, 0.85f,
                                         40000ULL, 50000ULL, &out) == OPENFC_LT_ERR_OUTLIER_JUMP);
    /* Sustained occlusion across wrap (> 300 ms -> dt = 410001 us) re-acquires cleanly! */
    assert(openfc_compute_landing_target(&calib, &st, true, 20.0f, 220.0f, 30.0f, 30.0f, 0.85f,
                                         400000ULL, 410000ULL, &out) == OPENFC_LT_OK);
    assert(st.prev_capture_us == 400000ULL);
}

static void test_marker_detector(void) {
    uint8_t img[64U * 48U];
    openfc_marker_cfg_t cfg = openfc_marker_default_cfg(64U, 48U);
    openfc_marker_result_t res;

    /* 1. Nominal marker */
    memset(img, 220, sizeof(img));
    for (uint16_t y = 14U; y < 26U; ++y) {
        for (uint16_t x = 36U; x < 48U; ++x) {
            img[y * 64U + x] = 20U;
        }
    }
    assert(openfc_detect_dark_marker(img, 64U, 48U, &cfg, &res));
    assert(res.detected);
    assert(fabsf(res.cx_px - 41.5f) < 0.6f);
    assert(fabsf(res.cy_px - 19.5f) < 0.6f);
    assert(res.area_px == 144U);

    /* 2. Marker flush against image border (0,0) */
    memset(img, 220, sizeof(img));
    for (uint16_t y = 0U; y < 12U; ++y) {
        for (uint16_t x = 0U; x < 12U; ++x) {
            img[y * 64U + x] = 25U;
        }
    }
    assert(openfc_detect_dark_marker(img, 64U, 48U, &cfg, &res));
    assert(fabsf(res.cx_px - 5.5f) < 0.6f);
    assert(fabsf(res.cy_px - 5.5f) < 0.6f);

    /* 3. Occluded (all black) & washed-out (all white) images rejected */
    memset(img, 10, sizeof(img));
    assert(!openfc_detect_dark_marker(img, 64U, 48U, &cfg, &res));
    memset(img, 245, sizeof(img));
    assert(!openfc_detect_dark_marker(img, 64U, 48U, &cfg, &res));

    /* 4. Low-contrast blur rejected */
    memset(img, 132, sizeof(img));
    for (uint16_t y = 14U; y < 26U; ++y) {
        for (uint16_t x = 26U; x < 38U; ++x) {
            img[y * 64U + x] = 112U;
        }
    }
    assert(!openfc_detect_dark_marker(img, 64U, 48U, &cfg, &res));

    /* 5. Thin horizontal streak rejected (aspect ratio) */
    memset(img, 220, sizeof(img));
    for (uint16_t y = 20U; y < 22U; ++y) {
        for (uint16_t x = 4U; x < 56U; ++x) {
            img[y * 64U + x] = 20U;
        }
    }
    assert(!openfc_detect_dark_marker(img, 64U, 48U, &cfg, &res));

    /* 6. Salt-and-pepper noise rejected (sparse bbox fill ratio) */
    memset(img, 220, sizeof(img));
    for (uint32_t i = 0U; i < sizeof(img); i += 19U) {
        img[i] = 15U;
    }
    assert(!openfc_detect_dark_marker(img, 64U, 48U, &cfg, &res));

    /* 7. Invalid config guards (min_area_px=0, inverted min/max, NaN ratios) */
    memset(img, 220, sizeof(img));
    for (uint16_t y = 14U; y < 26U; ++y) {
        for (uint16_t x = 36U; x < 48U; ++x) {
            img[y * 64U + x] = 20U;
        }
    }
    openfc_marker_cfg_t bad_cfg = cfg;
    bad_cfg.min_area_px = 0U;
    assert(!openfc_detect_dark_marker(img, 64U, 48U, &bad_cfg, &res));
    bad_cfg = cfg;
    bad_cfg.max_area_px = cfg.min_area_px - 1U;
    assert(!openfc_detect_dark_marker(img, 64U, 48U, &bad_cfg, &res));
    bad_cfg = cfg;
    bad_cfg.min_fill_ratio = NAN;
    assert(!openfc_detect_dark_marker(img, 64U, 48U, &bad_cfg, &res));
    assert(!openfc_detect_dark_marker(img, 4U, 4U, &cfg, &res));
    assert(!openfc_detect_dark_marker(NULL, 64U, 48U, &cfg, &res));
}

int main(void) {
    test_mavlink2_golden_vectors_and_guards();
    test_yaw_rotations_against_ardupilot();
    test_rejection_gates_wrap_and_occlusion_recovery();
    test_marker_detector();
    puts("test_vision_pipeline: PASS");
    return 0;
}
