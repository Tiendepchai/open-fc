#ifndef OPENFC_MARKER_DETECTOR_H
#define OPENFC_MARKER_DETECTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t min_contrast;     /* Minimum (bg_mean - dark_mean) luminance delta (0..255) */
    uint32_t min_area_px;     /* Minimum connected dark marker area in pixels */
    uint32_t max_area_px;     /* Maximum marker area in pixels (rejects full-frame blackout) */
    float min_fill_ratio;     /* area / (bbox_w * bbox_h) lower bound (rejects thin noise) */
    float max_aspect_ratio;   /* max(w/h, h/w) upper bound (rejects non-square streaks) */
} openfc_marker_cfg_t;

typedef struct {
    bool detected;
    float cx_px;
    float cy_px;
    float width_px;
    float height_px;
    uint32_t area_px;
    uint8_t contrast;
    float confidence;         /* 0.0 .. 1.0 heuristic score */
} openfc_marker_result_t;

openfc_marker_cfg_t openfc_marker_default_cfg(uint16_t img_w, uint16_t img_h);

/**
 * Detect a high-contrast dark fiducial marker on a lighter landing pad in an 8-bit
 * grayscale image without dynamic heap allocation.
 * Rejects occluded/dark frames, washed-out frames, low-contrast blur, and streaks.
 */
bool openfc_detect_dark_marker(
    const uint8_t *gray,
    uint16_t width,
    uint16_t height,
    const openfc_marker_cfg_t *cfg,
    openfc_marker_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* OPENFC_MARKER_DETECTOR_H */
