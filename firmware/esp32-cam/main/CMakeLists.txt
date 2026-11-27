idf_component_register(
    SRCS "cam_main.c" "marker_detector.c" "landing_target_math.c" "mavlink2_tx.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp32-camera
)
