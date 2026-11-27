/*
 * OPEN-FC MINIMAL v1.0 — esp32-cam OV2640 Vision Precision Landing Pipeline
 *
 * Safety & Epistemic Boundaries (AGENTS.md §1.1, §3.2, §4.4):
 * - Emits ONLY MAVLink2 HEARTBEAT (1 Hz) and validated LANDING_TARGET (#149).
 * - Never drives ESCs or sends flight-mode/arming commands.
 * - Refuses to emit LANDING_TARGET if camera intrinsics FX/FY are uncalibrated (0),
 *   if frame age exceeds CONFIG_OPENFC_CAM_MAX_FRAME_AGE_MS, or if marker fails checks.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "landing_target_math.h"
#include "marker_detector.h"
#include "mavlink2_tx.h"

static const char *TAG = "openfc_cam";

/* Candidate AI-Thinker ESP32-CAM OV2640 pinmap — verify against physical module in M3 */
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

static esp_err_t init_camera(void) {
    camera_config_t cfg = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_GRAYSCALE,
        .frame_size = FRAMESIZE_QVGA, /* Candidate 320x240 grayscale; benchmark FPS in M3 */
        .jpeg_quality = 12,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    return esp_camera_init(&cfg);
}

static void init_uart(void) {
    const uart_port_t port = (uart_port_t)CONFIG_OPENFC_CAM_UART_PORT;
    const uart_config_t cfg = {
        .baud_rate = CONFIG_OPENFC_CAM_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(port, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, CONFIG_OPENFC_CAM_UART_TX_GPIO, CONFIG_OPENFC_CAM_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static openfc_cam_calib_t load_calib_from_kconfig(void) {
    openfc_cam_calib_t c;
    c.img_width = 320U;
    c.img_height = 240U;
    c.fx = (float)CONFIG_OPENFC_CAM_FX_MILLIPX * 0.001f;
    c.fy = (float)CONFIG_OPENFC_CAM_FY_MILLIPX * 0.001f;
    c.cx = (float)CONFIG_OPENFC_CAM_CX_MILLIPX * 0.001f;
    c.cy = (float)CONFIG_OPENFC_CAM_CY_MILLIPX * 0.001f;
    c.cam_yaw_offset_rad = ((float)CONFIG_OPENFC_CAM_YAW_OFFSET_MDEG * 0.001f) * (3.14159265f / 180.0f);
    c.min_confidence = 0.15f;
    c.max_frame_age_us = (uint64_t)CONFIG_OPENFC_CAM_MAX_FRAME_AGE_MS * 1000ULL;
    c.max_step_rad = 0.45f;
    c.outlier_window_us = 300000ULL;
    return c;
}

static uint64_t monotonic_time_us(void) {
    int64_t t = esp_timer_get_time();
    return (t > 0) ? (uint64_t)t : 0ULL;
}

void app_main(void) {
    init_uart();

    openfc_mavlink2_tx_t mav;
    openfc_mavlink2_tx_init(&mav, (uint8_t)CONFIG_OPENFC_CAM_MAV_SYS_ID,
                            (uint8_t)CONFIG_OPENFC_CAM_MAV_COMP_ID);

    const uart_port_t port = (uart_port_t)CONFIG_OPENFC_CAM_UART_PORT;
    openfc_cam_calib_t calib = load_calib_from_kconfig();
    openfc_lt_filter_state_t filter;
    openfc_lt_filter_reset(&filter);

    const bool calib_ready = (calib.fx > 0.0f && calib.fy > 0.0f);
    if (!calib_ready) {
        ESP_LOGW(TAG, "Camera intrinsics UNCALIBRATED (fx/fy=0): LANDING_TARGET TX disabled until M3 calibration.");
    }

    bool cam_ok = (init_camera() == ESP_OK);
    if (!cam_ok) {
        ESP_LOGE(TAG, "OV2640 init failed; continuing 1 Hz HEARTBEAT only (no LANDING_TARGET).");
    }

    bool hb_sent = false;
    uint64_t last_hb_us = 0ULL;
    uint32_t frames_sent = 0U;
    uint32_t frames_dropped = 0U;
    uint8_t tx_buf[OPENFC_MAVLINK2_MAX_TX_BUF];

    for (;;) {
        uint64_t now_us = monotonic_time_us();
        if (!hb_sent || (now_us - last_hb_us) >= 1000000ULL) {
            uint8_t sys_state = (cam_ok && calib_ready) ? OPENFC_MAV_STATE_ACTIVE : OPENFC_MAV_STATE_STANDBY;
            size_t hb_len = openfc_mavlink2_pack_heartbeat(&mav, sys_state, tx_buf, sizeof(tx_buf));
            if (hb_len > 0U) {
                (void)uart_write_bytes(port, tx_buf, hb_len);
            }
            /* Drain unsolicited FC UART RX bytes so the unused RX ring stays clear */
            (void)uart_flush_input(port);
            hb_sent = true;
            last_hb_us = now_us;
            ESP_LOGI(TAG, "status: sent=%lu dropped=%lu", (unsigned long)frames_sent,
                     (unsigned long)frames_dropped);
        }

        if (!cam_ok) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            frames_dropped++;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        uint64_t capture_us = monotonic_time_us();

        openfc_marker_result_t det;
        memset(&det, 0, sizeof(det));
        bool fb_valid = (fb->buf != NULL) &&
                        (fb->format == PIXFORMAT_GRAYSCALE) &&
                        (fb->width == calib.img_width) &&
                        (fb->height == calib.img_height) &&
                        (fb->len >= ((size_t)fb->width * (size_t)fb->height));
        bool found = false;
        if (fb_valid) {
            openfc_marker_cfg_t det_cfg = openfc_marker_default_cfg((uint16_t)fb->width, (uint16_t)fb->height);
            found = openfc_detect_dark_marker(fb->buf, (uint16_t)fb->width, (uint16_t)fb->height,
                                              &det_cfg, &det);
        }
        esp_camera_fb_return(fb);

        uint64_t post_us = monotonic_time_us();
        openfc_lt_measurement_t meas;
        openfc_lt_status_t st = openfc_compute_landing_target(
            &calib, &filter, found, det.cx_px, det.cy_px, det.width_px, det.height_px,
            det.confidence, capture_us, post_us, &meas);

        if (st == OPENFC_LT_OK) {
            size_t n = openfc_mavlink2_pack_landing_target(&mav, &meas, tx_buf, sizeof(tx_buf));
            if (n > 0U && uart_write_bytes(port, tx_buf, n) == (int)n) {
                frames_sent++;
            } else {
                frames_dropped++;
            }
        } else {
            frames_dropped++;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
