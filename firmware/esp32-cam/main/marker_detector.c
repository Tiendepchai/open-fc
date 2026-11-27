#include "marker_detector.h"

#include <math.h>
#include <string.h>

openfc_marker_cfg_t openfc_marker_default_cfg(uint16_t img_w, uint16_t img_h) {
    uint32_t total = (uint32_t)img_w * (uint32_t)img_h;
    openfc_marker_cfg_t cfg;
    cfg.min_contrast = 50U;
    cfg.min_area_px = (total / 400U) > 16U ? (total / 400U) : 16U;
    cfg.max_area_px = total / 4U;
    cfg.min_fill_ratio = 0.45f;
    cfg.max_aspect_ratio = 1.65f;
    return cfg;
}

bool openfc_detect_dark_marker(
    const uint8_t *gray,
    uint16_t width,
    uint16_t height,
    const openfc_marker_cfg_t *cfg,
    openfc_marker_result_t *out) {
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
    if (gray == NULL || cfg == NULL || out == NULL || width < 8U || height < 8U) {
        return false;
    }
    if (cfg->min_area_px == 0U || cfg->max_area_px < cfg->min_area_px ||
        !isfinite(cfg->min_fill_ratio) || !isfinite(cfg->max_aspect_ratio) ||
        cfg->min_fill_ratio <= 0.0f || cfg->max_aspect_ratio < 1.0f) {
        return false;
    }

    const uint32_t n_px = (uint32_t)width * (uint32_t)height;
    uint64_t sum_all = 0U;
    uint8_t min_val = 255U;
    uint8_t max_val = 0U;
    for (uint32_t i = 0U; i < n_px; ++i) {
        uint8_t p = gray[i];
        sum_all += p;
        if (p < min_val) {
            min_val = p;
        }
        if (p > max_val) {
            max_val = p;
        }
    }

    if ((uint8_t)(max_val - min_val) < cfg->min_contrast) {
        return false;
    }

    uint32_t mean_val = (uint32_t)(sum_all / n_px);
    /* Adaptive dark threshold halfway between global mean and darkest pixel */
    uint8_t thresh = (uint8_t)((mean_val + (uint32_t)min_val) / 2U);

    uint64_t sum_x = 0U;
    uint64_t sum_y = 0U;
    uint64_t sum_dark = 0U;
    uint64_t sum_light = 0U;
    uint32_t area = 0U;
    uint32_t light_cnt = 0U;
    uint16_t min_x = width;
    uint16_t max_x = 0U;
    uint16_t min_y = height;
    uint16_t max_y = 0U;

    for (uint16_t y = 0U; y < height; ++y) {
        const uint8_t *row = gray + ((size_t)y * (size_t)width);
        for (uint16_t x = 0U; x < width; ++x) {
            uint8_t p = row[x];
            if (p <= thresh) {
                sum_x += x;
                sum_y += y;
                sum_dark += p;
                area++;
                if (x < min_x) {
                    min_x = x;
                }
                if (x > max_x) {
                    max_x = x;
                }
                if (y < min_y) {
                    min_y = y;
                }
                if (y > max_y) {
                    max_y = y;
                }
            } else {
                sum_light += p;
                light_cnt++;
            }
        }
    }

    if (area == 0U || area < cfg->min_area_px || area > cfg->max_area_px || light_cnt == 0U) {
        return false;
    }

    uint32_t dark_mean = (uint32_t)(sum_dark / area);
    uint32_t light_mean = (uint32_t)(sum_light / light_cnt);
    if (light_mean <= dark_mean || (light_mean - dark_mean) < cfg->min_contrast) {
        return false;
    }

    float bw = (float)((uint32_t)(max_x - min_x) + 1U);
    float bh = (float)((uint32_t)(max_y - min_y) + 1U);
    float bbox_area = bw * bh;
    if (bbox_area <= 0.0f) {
        return false;
    }

    float fill_ratio = (float)area / bbox_area;
    float aspect = (bw >= bh) ? (bw / bh) : (bh / bw);
    if (fill_ratio < cfg->min_fill_ratio || aspect > cfg->max_aspect_ratio) {
        return false;
    }

    uint8_t contrast = (uint8_t)(light_mean - dark_mean);
    float contrast_score = (float)contrast / 255.0f;
    float shape_score = (fill_ratio > 1.0f) ? 1.0f : fill_ratio;

    out->detected = true;
    /* Double-precision division prevents float32 rounding drift when sum_x/sum_y > 2^24 */
    out->cx_px = (float)((double)sum_x / (double)area);
    out->cy_px = (float)((double)sum_y / (double)area);
    out->width_px = bw;
    out->height_px = bh;
    out->area_px = area;
    out->contrast = contrast;
    out->confidence = contrast_score * shape_score;
    return true;
}
