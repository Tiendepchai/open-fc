# AGENTS.md — OPEN-FC MINIMAL v1.0 Engineering Guardrails

> **Scope:** Applies to all AI coding agents, automation scripts, and human contributors working in this repository.
> **Current Project State:** `HARDWARE DESIGN (REV K) & M0 VERIFICATION — BENCH ONLY`. Not flight-certified.

## 1. Mission & Non-Negotiable Safety Boundaries

1. **Single Flight Authority:** `WeAct STM32F405RGT6` running `ArduCopter / ChibiOS` (`OPENFCF405`) is the sole flight-critical controller. Companion boards (`ESP32 DevKit`, `ESP32-CAM`) MUST NEVER drive ESCs or bypass FC arming, EKF, or failsafe logic.
2. **Hard Stop — No Flight Readiness Claims:** Never declare the system flight-ready based on successful compilation, KiCad ERC/DRC pass, or SITL pass. All bench tests require propellers removed.
3. **Preserve Safety Features Under Flash Pressure:** STM32F405 has a strict **1 MB Flash** ceiling. If `OPENFCF405` firmware overflows Flash, document exact build size and disable only non-baseline features (e.g., CAN, OSD, Lua, Mount, Sprayer, unused sensor drivers). **NEVER** remove EKF, pre-arm checks, RC/GPS/battery failsafes, or MAVLink Precision Landing backend just to make a build pass.
4. **No Secrets in Repo:** Never commit Wi-Fi SSIDs/passwords, API tokens, SSH keys, or GCS credentials. Use `sdkconfig.defaults` with placeholder values and keep real secrets in untracked local files.

## 2. Frozen Hardware Baseline (Carrier PCB Rev K — 100% THT)

Do NOT re-introduce superseded parts (`TMUX1511PW`, `Adapter TSSOP14`, `100 kΩ` VBAT pull-down, `KF2510`, `RuiBet 55A`, `TIM8 PC6–PC9`, external SPI CS pull-ups) without a formal Decision Record in `docs/decisions/`.

### 2.1. Verified MCU Pinmap (`U1: WeAct STM32F405RGT6`)
| Subsystem | Peripheral | STM32 GPIO | Carrier Interface | Verified Status |
|---|---|---|---|---|
| IMU (`MPU6500`) | `SPI1` + `EXTI` | `PA5/PA6/PA7`, `PA4 (CS)`, `PC4 (INT)` | `U2` (`1x10` Socket) | Measured (`~9 kΩ` internal CS pull-up; `FSYNC=GND`) |
| Baro (`BMP280`) | `SPI1` | `PA5/PA6/PA7`, `PB12 (CS)` | `U3` (`1x6` Socket) | Measured (`~9 kΩ` internal CS pull-up) |
| Wi-Fi (`ESP32 DevKit`) | `USART1` | `PA9 (TX)`, `PA10 (RX)` | `J_WIFI` (`JST-XH 4P`) | `PA9` verified isolated from `5V/VBUS` |
| Vision (`ESP32-CAM`) | `USART2` | `PA2 (TX)`, `PA3 (RX)` | `J_CAM` (`JST-XH 4P`) | Pinout identical to `J_WIFI` (`1:+5V_COMP, 2:GND, 3:TX, 4:RX`) |
| GPS + Compass (`BN-880`) | `USART6` + `I2C2` | `PC6/PC7`, `PB10 (SCL)`, `PB11 (SDA)` | `J_GPS` (`JST-XH 6P`) | Measured `18 kΩ` on module; `R9/R10 4.7 kΩ` to `+3V3` on PCB |
| RC (`MC7RE`) | `UART4` | `PA1 (RX)` | `J_RC` (`JST-XH 3P`) | `2N3904` (`Q1`) inverter + `JP_SBUS` (`1-2` default INV) |
| ESC (`Uangel 60A 4-in-1`) | `TIM4` | `PB6/PB7/PB8/PB9` | `J_ESC` (`JST-XH 8P`) | `10 kΩ` pull-down (`R5-R8`) + `47 Ω` series (`R1-R4`) |
| Battery Voltage | `ADC1_IN0` | `PA0` | `J_ESC Pin 2` (`VBAT_RAW`) | `220 kΩ (1%) / 18 kΩ (1%)` + `4.7 kΩ` + `100 nF` (`MULT ≈ 13.22`) |
| Current Sense (Optional) | `ADC1_IN10` | `PC0` | `J_ESC Pin 7` (`CURR_RAW`) | `JP_C` **DEFAULT OPEN** + `10 kΩ` (`R15`); do not close uncharacterized |

### 2.2. Power Architecture (Option 1 — Pixhawk Style)
- **External UBEC `5V/3A`:** Input soldered directly to ESC main `BAT+ / BAT-` pads (20–22 AWG); output enters `J_PWR` (`JST-XH 2P`).
- **Star Routing:**
  - `+5V_COMP` (direct from `J_PWR` + `C1/C2 470 µF`): Powers `J_WIFI` and `J_CAM`.
  - `+5V_FC` (through Schottky `D1: 1N5822` + `C6 100 nF`): Powers `U1 (5V)`, `J_GPS`, `J_RC` and blocks USB-C backfeed.
- **Safety Rule:** `J_ESC Pin 2 = VBAT_RAW (3S, up to 12.6 V)`. Never route `VBAT_RAW` to `5V`, `3V3`, or directly to any MCU pin. Carrier PCB carries signal/logic currents only—never motor phase or high-current battery paths.

## 3. Epistemic & Documentation Rules

1. **Strict State Labeling:** Every hardware, firmware, or performance claim MUST be tagged with its actual status:
   - `confirmed`: Verified by user/decision log.
   - `measured`: Backed by multimeter/oscilloscope/build log evidence in `docs/hw-evidence.md` or `reports/`.
   - `design`: Specified in KiCad schematic/PCB but not yet fabricated or tested.
   - `candidate`: Proposed default (e.g., DShot vs PWM, UART baudrate, FPS target) pending validation.
   - `validated`: Passed acceptance gate tests.
2. **Never Fabricate Pinouts or Benchmarks:** Do not infer physical header pin numbers from similar boards or guess ESP32-CAM FPS/landing accuracy from OV2640 datasheets.
3. **Keep Artifacts Synchronized:** Any change to pin assignments, component values, or connector policies MUST update all affected files in the same commit:
   - `docs/pinmap-verified.csv`
   - `docs/bom-rev-k.csv`
   - `hardware/kicad/open-fc-carrier-rev-k/*`
   - `firmware/board-openfcf405/hwdef.dat`
   - `docs/OPEN_FC_MINIMAL_v1.0.md` (Decision Log)

## 4. Firmware & Toolchain Workflow Rules

1. **Upstream Pinning:** Keep `firmware/ardupilot` pinned to a specific stable tag/commit. Place all custom board definitions in `firmware/board-openfcf405/` and avoid untracked edits inside the submodule.
2. **TIM4 DMA Verification Required:** On STM32F405, `TIM4_CH4 (PB9)` lacks a dedicated compare DMA stream. Before enabling DShot in `hwdef.dat`, verify ChibiOS DMA allocation (`scripts/check-dma.py`) and bench-test with the Uangel 60A ESC. Fall back to `PWM` / `OneShot125` if DMA contention occurs.
3. **Motor Mapping in Software:** PCB routes `PB6–PB9` straight to `J_ESC Pin 3–6` without trace crossovers. Map physical F450 Quad-X corners via ArduCopter `SERVO1_FUNCTION`–`SERVO4_FUNCTION` during prop-off motor tests.
4. **Companion Isolation:** `esp32-wifi` and `esp32-cam` use pinned `ESP-IDF`. Both must handle UART framing errors, link disconnects, and stale timestamps cleanly without blocking or sending unvalidated setpoints to the FC.

## 5. Checklist Before Submitting Changes

- [ ] Does the change respect the Rev K 100% THT & `JST-XH` baseline?
- [ ] Are `JP_C` (OPEN) and `VBAT_RAW` isolation rules intact?
- [ ] If firmware was modified, was Flash/RAM usage logged in `reports/`?
- [ ] Are all claims accurately labeled (`measured` vs `design` vs `candidate`) without leaking secrets?
