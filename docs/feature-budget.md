# Feature Budget & DMA Allocation (`OPENFCF405`)

> **Target MCU:** `STM32F405RGT6` (`1024 KB` total Flash; `32 KB` bootloader `0x08000000` + `32 KB` dual-page `AP_FlashStorage` `0x08008000` → **`960 KB` app slot** `0x08010000`).
> **Pinned Upstream:** `ArduPilot Copter-4.6.3` (`92b0cd788ec29406f26c6f9c31d5ceedbd1cc538`).
> **Toolchain:** Official `Arm GNU Toolchain 12.3.1 (arm-12.35)` (`darwin-arm64`, SHA-256 verified in `.toolchain/`).
> **Status:** M1 firmware build, 1 MB Flash ceiling, bootloader `<= 32 KB` gate, ROMFS `defaults.parm` embedding, ELF safety symbol audit, and `TIM4` DMA resolution **`measured` / `PASS`**.

## 1. Flash & RAM Budget (`measured`)

- **Bootloader (`AP_Bootloader.bin`, `0x08000000`, `32 KB` slot):**
  - Used: **`14,080 bytes`** (`13.75 KB`) / `32,768 bytes` (**`18,688 bytes` free**; ELF text `14,012 B`, data `64 B`).
- **Dual-page parameter storage (`STORAGE_FLASH_PAGE 2`):**
  - Reserved: `0x08008000–0x0800FFFF` (`32,768 bytes`, `Sectors 2–3`, `HAL_STORAGE_SIZE = 15360`).
- **ArduCopter Application (`arducopter.bin`, `0x08010000`, `960 KB` slot):**
  - **Text:** `734,853 bytes` | **Data:** `956 bytes` | **BSS (SRAM0):** `130,208 bytes`
  - **Total Flash Used (`arducopter.bin`):** **`735,816 bytes` (`718.6 KB`)** / `983,040 bytes` (`960 KB`)
  - **Free Flash Headroom:** **`247,224 bytes` (`241.4 KB`, 25.1% margin)**
  - **ROMFS Embedded (`ap_romfs_embedded.h`):** `defaults.parm` (`2,196 B` uncompressed), `bootloader.bin` (`14,080 B`), `hwdef.dat` (`AP_PARAM_DEFAULTS_FILE_PARSING_ENABLED = 1`).

### Build Size Log

| Date | ArduPilot Tag / Commit | Bootloader (B) | App Flash Used (B) | App Slot (B) | Headroom (B) | Verdict | Report |
|---|---|---|---|---|---|---|---|
| `2026-11-26` | `Copter-4.6.3` (`92b0cd78`) | `14,080 / 32,768` | `734,584` | `983,040` | `248,456` | `PASS` | [build-openfcf405-20261126-090258Z.log](../reports/build-openfcf405-20261126-090258Z.log) |
| `2026-11-26` | `Copter-4.6.3` (`92b0cd78`) + `QMC5883P` + hardened `defaults.parm` & `DEFAULT_SERIAL*` | `14,080 / 32,768` | `735,816` | `983,040` | **`247,224`** | **`PASS`** | [build-openfcf405-20261126-093716Z.log](../reports/build-openfcf405-20261126-093716Z.log) |

### Mandatory Safety & Baseline ELF Symbol Proof (`measured` via `arm-none-eabi-nm --defined-only -C`)

| Capability (`AGENTS.md §1.3`) | Verified Defined Symbol(s) | Status |
|---|---|---|
| EKF3 | `NavEKF3::InitialiseFilter` (1) | `PASS` |
| Pre-arm checks | `AP_Arming::pre_arm_checks` (1) | `PASS` |
| RC failsafe | `Copter::failsafe_radio_on_event` (1) | `PASS` |
| Battery failsafe | `Copter::handle_battery_failsafe` (2) | `PASS` |
| EKF & GCS failsafes | `Copter::failsafe_ekf_event` (1), `Copter::failsafe_gcs_check` (1) | `PASS` |
| MAVLink2 Precision Landing | `AC_PrecLand_Companion` (7) | `PASS` |
| IMU (`MPU6500` SPI1) | `AP_InertialSensor_Invensense` (29) | `PASS` |
| Baro (`BMP280` SPI1) | `AP_Baro_BMP280` (13) | `PASS` |
| GPS (`BN-880` USART6) | `AP_GPS_UBLOX` (47) | `PASS` |
| Compass (`BN-880` I2C2) | `AP_Compass_HMC5843` (16), `AP_Compass_QMC5883L` (10), `AP_Compass_QMC5883P` (10) | `PASS` |
| RC (`MC7RE` SBUS UART4) | `AP_RCProtocol_SBUS` (10) | `PASS` |
| `TIM4_UP` DMAR DShot engine | `ChibiOS::RCOutput::send_pulses_DMAR` (1) | `PASS` |
| ROMFS `defaults.parm` loader | `AP_Param::reload_defaults_file` (1), `AP_ROMFS::find_decompress` (1) | `PASS` |

- **Non-baseline features disabled in `hwdef.dat`:** `include ../include/minimize_features.inc`, DroneCAN, Lua scripting, OSD, Mount, Sprayer, Camera, Terrain, OpticalFlow, Rangefinder, ADSB, Parachute, Airspeed (+ 6 backends `undef`'d from `minimize_common.inc`), unused IMU/Baro drivers.
- **Rev K hardware drivers explicitly preserved/enabled:** `AP_INERTIALSENSOR_INVENSENSE_ENABLED 1` (`MPU6500`), `AP_BARO_BMP280_ENABLED 1` (`BMP280`), `AP_GPS_UBLOX_ENABLED 1` (`BN-880`), `AP_COMPASS_QMC5883P_ENABLED 1` (alongside `HMC5843`/`QMC5883L` for modern `BN-880` units), `AC_PRECLAND_ENABLED 1`, `AP_FENCE_ENABLED 1`.

## 2. ChibiOS `TIM4` DMA Resolution (`measured` via `chibios_hwdef.py` + `build-fc.sh`)

Evaluated 3 configurations against `ArduPilot Copter-4.6.3`:

| Case | `hwdef.dat` Setting | `SHARED_DMA_MASK` | `TIM4_UP` DMA | Motor Protocol Support | Verdict |
|---|---|---|---|---|---|
| **A** | `PB6–PB9 NODMA`, `USART2_TX` DMA | `0` | None | Normal PWM / OneShot125 only | Valid fallback |
| **B** | `PB6–PB9` DMA, `USART2_TX` DMA | `DMA1 S6` shared | Shared w/ `USART2_TX` | Contention jitter on `TIM4_UP` | **REJECTED** |
| **C (Locked)** | `PB6–PB9` DMAR, `PA2 USART2_TX NODMA`, `DMA_NOSHARE TIM4_UP SPI1*` | **`0` (ZERO sharing)** | **Dedicated `DMA1 S6 C2`** | **PWM + OneShot125 + Unidirectional DShot150/300/600** | **LOCKED (`PASS`)** |

### Verified `Copter-4.6.3` DMA Map (Case C — `SHARED_DMA_MASK = 0`)

| DMA Controller / Stream | Peripheral | Channel | Notes |
|---|---|---|---|
| `DMA1 Stream 2` | `UART_UART4_RX` (`PA1`) | `4` | `J_RC` (`MC7RE` SBUS via `Q1 2N3904`; RX-only) |
| `DMA1 Stream 3` | `I2C_I2C2_RX` (`PB10/PB11`) | `7` | `J_GPS` (`BN-880` external compass) |
| `DMA1 Stream 5` | `UART_USART2_RX` (`PA3`) | `4` | `J_CAM` (`ESP32-CAM` incoming `LANDING_TARGET`) |
| `DMA1 Stream 6` | **`TIM_TIM4_UP`** (`PB6–PB9`) | `2` | **Dedicated 4-ch DMAR burst (`CCR1..CCR4`)** |
| `DMA1 Stream 7` | `I2C_I2C2_TX` (`PB10/PB11`) | `7` | `J_GPS` (`BN-880` external compass) |
| `DMA2 Stream 0` | `ADC_ADC1` (`PA0`/`PC0`) | `0` | Battery voltage (`PA0`, `ADC_SAMPLE_480`); `PC0` disabled while `JP_C` OPEN |
| `DMA2 Stream 1` | `UART_USART6_RX` (`PC7`) | `5` | `J_GPS` (`BN-880` GPS) |
| `DMA2 Stream 2` | `SPI_SPI1_RX` (`PA6`) | `3` | `MPU6500` + `BMP280` (`5.25 MHz` high / `656 kHz` low) |
| `DMA2 Stream 3` | `SPI_SPI1_TX` (`PA7`) | `3` | `MPU6500` + `BMP280` |
| `DMA2 Stream 5` | `UART_USART1_RX` (`PA10`) | `4` | `J_WIFI` (`ESP32 DevKit` MAVLink2) |
| `DMA2 Stream 6` | `UART_USART6_TX` (`PC6`) | `5` | `J_GPS` (`BN-880` GPS) |
| `DMA2 Stream 7` | `UART_USART1_TX` (`PA9`) | `4` | `J_WIFI` (`ESP32 DevKit` MAVLink2; `PA9` isolated from USB VBUS) |
| IRQ/FIFO (`NODMA`) | `UART_USART2_TX` (`PA2`) | — | Low-rate FC→`ESP32-CAM` TX (`115200` baud) |

### Motor Protocol Decision (`candidate` → validate prop-off on Uangel 60A ESC in M2)
- **Firmware capability (`measured`):** Unidirectional DShot150/300/600 (`TIM4_UP` DMAR bursts all 4 `CCR1–CCR4` including `PB9`), OneShot (`1`), OneShot125 (`2`), and Normal PWM (`0`) all run with `SHARED_DMA_MASK = 0`.
- **Bi-Directional DShot (`BIDIR`):** **NOT supported** (`PB9 / TIM4_CH4` lacks a compare/capture DMA stream on STM32F405, and `TIM4_CH2/CH3` compare streams collide with `I2C2`).
- **`defaults.parm` initial setting:** `MOT_PWM_TYPE 0` (Normal PWM) for safest M2 prop-off bring-up; switch to `MOT_PWM_TYPE 5` (DShot300) via parameter without reflashing once Uangel 60A ESC passes prop-off bench verification.
- **Rangefinder-less Precision Landing (`candidate`):** `PLND_ALT_MIN 0` and `PLND_ALT_MAX 0` are set alongside `PLND_ENABLED 1`, `PLND_TYPE 1`, `PLND_STRICT 1` because `AP_RANGEFINDER_ENABLED=0` on Rev K (non-zero defaults would force `TARGET_OUT_OF_RANGE` whenever `!rangefinder_alt_valid`).
