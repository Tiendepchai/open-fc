cmake_minimum_required(VERSION 3.16)

# Pinned ESP-IDF v5.2+ project — OPEN-FC MINIMAL v1.0 (M2 Wi-Fi Telemetry Bridge)
# Companion isolation (AGENTS.md §1.1): transparent MAVLink2 UDP <-> UART1 forwarder only.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(openfc_esp32_wifi)
