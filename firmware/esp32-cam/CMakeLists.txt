cmake_minimum_required(VERSION 3.16)

# Pinned ESP-IDF v5.2+ project — OPEN-FC MINIMAL v1.0 (M3 Vision Precision Landing)
# Companion isolation (AGENTS.md §1.1): emits HEARTBEAT + validated LANDING_TARGET only.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(openfc_esp32_cam)
