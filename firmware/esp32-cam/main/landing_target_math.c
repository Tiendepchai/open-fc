#include "landing_target_math.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

void openfc_lt_filter_reset(openfc_lt_filter_state_t *st) {
    if (st != NULL) {
        memset(st, 0, sizeof(*st));
    }
}

openfc_lt_status_t openfc_compute_landing_target(
    const openfc_cam_calib_t *calib,
    openfc_lt_filter_state_t *filter,
    bool detected,
    float u_px,
    float v_px,
    float box_w_px,
    float box_h_px,
    float confidence,
    uint64_t capture_us,
    uint64_t now_us,
    openfc_lt_measurement_t *out) {
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
    if (calib == NULL || out == NULL) {
        return OPENFC_LT_ERR_UNCALIBRATED;
    }
    if (!isfinite(calib->fx) || !isfinite(calib->fy) || !isfinite(calib->cx) || !isfinite(calib->cy) ||
        !isfinite(calib->cam_yaw_offset_rad) || !isfinite(calib->min_confidence) ||
        !isfinite(calib->max_step_rad) || calib->fx <= 0.0f || calib->fy <= 0.0f ||
        calib->min_confidence < 0.0f || calib->min_confidence > 1.0f || calib->max_step_rad < 0.0f ||
        calib->img_width == 0U || calib->img_height == 0U) {
        return OPENFC_LT_ERR_UNCALIBRATED;
    }
    if (!detected) {
        return OPENFC_LT_ERR_NOT_DETECTED;
    }
    if (!isfinite(u_px) || !isfinite(v_px) || !isfinite(box_w_px) || !isfinite(box_h_px) ||
        !isfinite(confidence)) {
        return OPENFC_LT_ERR_NON_FINITE;
    }
    if (u_px < 0.0f || u_px >= (float)calib->img_width ||
        v_px < 0.0f || v_px >= (float)calib->img_height) {
        return OPENFC_LT_ERR_OUT_OF_BOUNDS;
    }
    if (confidence < calib->min_confidence) {
        return OPENFC_LT_ERR_LOW_CONFIDENCE;
    }

    /*
     * Wrap-safe unsigned half-space age check:
     * - Forward wrap across UINT64_MAX yields exact age_us <= max_frame_age_us.
     * - Inverted/backwards now_us < capture_us yields age_us > UINT64_MAX/2 -> rejected.
     */
    const uint64_t half_u64 = UINT64_MAX / 2ULL;
    uint64_t age_us = now_us - capture_us;
    if (age_us > half_u64 || age_us > calib->max_frame_age_us) {
        return OPENFC_LT_ERR_STALE_FRAME;
    }

    /*
     * Camera optical ray (z_cam = +1 downward):
     *   cam_right   = (u - cx) / fx
     *   cam_forward = -(v - cy) / fy  (top of image v < cy corresponds to +X forward)
     * Rotate by cam_yaw_offset_rad into ArduPilot Body FRD (+X Forward, +Y Right, +Z Down):
     */
    float cam_right = (u_px - calib->cx) / calib->fx;
    float cam_fwd = -(v_px - calib->cy) / calib->fy;

    float c = cosf(calib->cam_yaw_offset_rad);
    float s = sinf(calib->cam_yaw_offset_rad);
    float body_fwd = (c * cam_fwd) - (s * cam_right);
    float body_right = (s * cam_fwd) + (c * cam_right);

    if (!isfinite(cam_right) || !isfinite(cam_fwd) || !isfinite(body_fwd) || !isfinite(body_right)) {
        return OPENFC_LT_ERR_NON_FINITE;
    }

    /*
     * Match ArduCopter AC_PrecLand_Companion.cpp:45:
     *   _los_meas_body = Vector3f(-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f);
     * Therefore:
     *   tan(angle_x) = +body_right  => angle_x = atan2f(body_right, 1.0f)
     *   tan(angle_y) = -body_fwd    => angle_y = atan2f(-body_fwd, 1.0f)
     */
    float angle_x = atan2f(body_right, 1.0f);
    float angle_y = atan2f(-body_fwd, 1.0f);

    if (!isfinite(angle_x) || !isfinite(angle_y)) {
        return OPENFC_LT_ERR_NON_FINITE;
    }

    if (filter != NULL && filter->has_prev) {
        /* Wrap-safe monotonic sequence delta; reject out-of-order older frames without corrupting filter */
        uint64_t dt = capture_us - filter->prev_capture_us;
        if (dt > half_u64) {
            return OPENFC_LT_ERR_STALE_FRAME;
        }
        if (calib->max_step_rad > 0.0f && dt <= calib->outlier_window_us) {
            float dx = fabsf(angle_x - filter->prev_angle_x);
            float dy = fabsf(angle_y - filter->prev_angle_y);
            if (dx > calib->max_step_rad || dy > calib->max_step_rad) {
                return OPENFC_LT_ERR_OUTLIER_JUMP;
            }
        }
    }

    float size_x = (box_w_px > 0.0f) ? (2.0f * atan2f(0.5f * box_w_px, calib->fx)) : 0.0f;
    float size_y = (box_h_px > 0.0f) ? (2.0f * atan2f(0.5f * box_h_px, calib->fy)) : 0.0f;
    if (!isfinite(size_x) || !isfinite(size_y)) {
        return OPENFC_LT_ERR_NON_FINITE;
    }

    if (filter != NULL) {
        filter->has_prev = true;
        filter->prev_capture_us = capture_us;
        filter->prev_angle_x = angle_x;
        filter->prev_angle_y = angle_y;
    }

    out->time_usec = capture_us;
    out->angle_x_rad = angle_x;
    out->angle_y_rad = angle_y;
    out->size_x_rad = size_x;
    out->size_y_rad = size_y;
    out->distance_m = 0.0f;
    out->position_valid = 0U;
    return OPENFC_LT_OK;
}
