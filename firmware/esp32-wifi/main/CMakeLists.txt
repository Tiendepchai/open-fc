idf_component_register(
    SRCS "bridge_main.c" "mavlink_framer.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_event esp_netif nvs_flash lwip
)
