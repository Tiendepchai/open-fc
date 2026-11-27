#ifndef OPENFC_LANDING_TARGET_MATH_H
#define OPENFC_LANDING_TARGET_MATH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OPENFC_LT_OK = 0,
    OPENFC_LT_ERR_UNCALIBRATED = 1,
    OPENFC_LT_ERR_NOT_DETECTED = 2,
    OPENFC_LT_ERR_OUT_OF_BOUNDS = 3,
    OPENFC_LT_ERR_LOW_CONFIDENCE = 4,
    OPENFC_LT_ERR_STALE_FRAME = 5,
    OPENFC_LT_ERR_OUTLIER_JUMP = 6,
    OPENFC_LT_ERR_NON_FINITE = 7
} openfc_lt_status_t;

typedef struct {
    uint16_t img_width;
    uint16_t img_height;
    float fx;                 /* Must be > 0 (0 = uncalibrated -> blocks TX) */
    float fy;                 /* Must be > 0 (0 = uncalibrated -> blocks TX) */
    float cx;
    float cy;
    float cam_yaw_offset_rad; /* Rotation from camera optical axes to FRD body */
    float min_confidence;
    uint64_t max_frame_age_us;
    float max_step_rad;       /* Outlier rejection threshold between adjacent frames */
    uint64_t outlier_window_us;
} openfc_cam_calib_t;

typedef struct {
    bool has_prev;
    uint64_t prev_capture_us;
    float prev_angle_x;
    float prev_angle_y;
} openfc_lt_filter_state_t;

typedef struct {
    uint64_t time_usec;
    float angle_x_rad;        /* ArduCopter AC_PrecLand_Companion.cpp:45 (+Y Right = +tan(angle_x)) */
    float angle_y_rad;        /* ArduCopter AC_PrecLand_Companion.cpp:45 (+X Forward = -tan(angle_y)) */
    float size_x_rad;
    float size_y_rad;
    float distance_m;         /* 0.0f when no LiDAR / uncalibrated depth */
    uint8_t position_valid;   /* 0 for pure angular PrecLand */
} openfc_lt_measurement_t;

void openfc_lt_filter_reset(openfc_lt_filter_state_t *st);

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
    openfc_lt_measurement_t *out);

#ifdef __cplusplus
}
#endif

#endif /* OPENFC_LANDING_TARGET_MATH_H */
