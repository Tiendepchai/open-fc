# OPEN-FC MINIMAL v1.0 — Master Project Baseline (Rev K — 100% THT)

> **Source of Truth:** System-level baseline locked to **Carrier PCB Rev K (100% THT)**.
> **Current State:** `HARDWARE DESIGN (REV K) & M0 VERIFICATION — BENCH ONLY`. Not flight-certified.
> **State Tags:** `confirmed` | `measured` | `design` | `candidate` | `validated`.

---

## 0. Project Goals & Definition of Done (v1.0 — `confirmed`)

### 0.1. North Star Goal (v1.0)
Build an open-source F450 research quadcopter running `ArduCopter` on `WeAct STM32F405RGT6` via **Carrier PCB Rev K (100% THT)**, strictly separating real-time flight control from Wi-Fi telemetry (`ESP32 DevKit`) and computer vision (`ESP32-CAM OV2640`), capable of stable flight, GPS position hold, full failsafe execution, and marker-based Precision Landing without LiDAR.

### 0.2. Immediate Goal (M0 & M1 Bottlenecks)
1. **Hardware (M0 & M1):** Complete KiCad Carrier PCB Rev K (`60 × 56 mm`, 100% THT, `JST-XH 2.50 mm`, Power Option 1) at **0 ERC/DRC errors**, 100% fit on 1:1 A4 printout, and complete the 3 remaining physical bench checks (`JST-XH` cable polarity `1→1` vs `1→N`, `MC7RE` SBUS waveform, UBEC `5V/3A` dual-ESP32 stress test).
2. **Firmware (M1):** Compile `OPENFCF405` on `ArduCopter` cleanly under the **1 MB Flash** ceiling while preserving EKF, GPS, Precision Landing, and all failsafes, and lock the motor protocol (`PWM` / `OneShot125` vs `DShot`) based on ChibiOS `TIM4` DMA allocation.

### 0.3. Definition of Done (v1.0)
v1.0 is complete ONLY when all 4 physical verification gates pass:
1. **Bench (Props OFF):** Clean reads of `MPU6500` (IMU), `BMP280` (Baro), `BN-880` (GPS/Compass), clean `SBUS` decode, calibrated `VBAT`, stable MAVLink2 over Wi-Fi, and fault-injection pass (RC pull, Wi-Fi disconnect, simulated battery sag).
2. **Vision:** `ESP32-CAM` outputs calibrated MAVLink2 `LANDING_TARGET` with bounded latency and automatic bad/stale/lost-marker rejection.
3. **Basic Flight:** Stable manual takeoff (`Stabilize` / `AltHold`) and GPS position hold (`Loiter`) on F450 with IMU vibration within safe limits.
4. **Autonomous Flight:** Successful marker-based Precision Landing test inside an authorized test area.

---

## 1. Rev K Hardware Delta (`confirmed` / `design`)

- **Carrier PCB (`design`):** `60 × 68 mm` (`Rev K.1`, ADR-003), 2-layer FR4, 4× M3 mounting holes (`45 × 45 mm`), 100% THT (`23` discrete parts + `12` headers/sockets = `35` electrical parts + `4` M3 holes). Open-sky `8.5 mm` sockets for `U1`, `U2 (25.10×14.88 mm)`, `U3 (15.38×11.50 mm)`; low-profile discretes (`≤ 5.5 mm`) nested under WeAct `U1` (`≥ 9.2 mm` clear). Replaces hand-wired perfboard plan.
- **Motor & Battery Voltage Block (`Sheet 1`, `confirmed` / `design`):**
  - Removed `TMUX1511PW`, `Adapter TSSOP14`, and `100 kΩ` VBAT pull-down.
  - Passive current-limited voltage divider into `PA0` (`ADC1_IN0`): `220 kΩ (1%) / 18 kΩ (1%)` + `4.7 kΩ` series + `100 nF` filter cap (`MULT ≈ 13.22`, unpowered FC backfeed `~33 µA`).
  - Independent `47 Ω` series (`R1–R4`) + `10 kΩ` pull-down (`R5–R8`) on `PB6–PB9` (`TIM4_CH1–CH4`).
  - `JP_C` (`PC0` current sense, `R15 10 kΩ`) **DEFAULT OPEN**; do not close until `CURR_RAW` is characterized.
- **Power (Option 1 — Pixhawk Style) & SBUS (`Sheet 2`, `confirmed` / `design`):**
  - External UBEC `5V/3A` input soldered directly to ESC Uangel 60A `BAT+ / BAT-` pads (20–22 AWG); output connects to `J_PWR (JST-XH 2P)`.
  - Star power split:
    - `+5V_COMP` (pre-`D1`, `C1/C2 470 µF`): powers `J_WIFI` and `J_CAM`.
    - `+5V_FC` (post-Schottky `D1 1N5822`, `C6 100 nF`): powers `U1 (5V)`, `J_GPS`, `J_RC`, blocking USB-C backfeed.
  - External inverted SBUS carrier PCB feeds `J_RC1.3` → series protection resistor `R12 (4.7 kΩ)` → `PA1 (UART4_RX)` (`Q1 2N3904`, `R11`, `R13`, `JP_SBUS1` removed per ADR-003).
- **Sensors & Peripherals (`Sheet 3`, `measured` / `design`):**
  - `MPU6500 (NCS)` and `BMP280 (CSB)` measured `~9 kΩ` internal pull-up → external SPI CS pull-ups removed.
  - `BN-880` measured `18 kΩ` on `SDA/SCL` → PCB adds `R9/R10 4.7 kΩ` pull-ups to `+3V3` for clean `400 kHz` I²C edges over GPS mast cable.
- **Connector Standard (`confirmed`):**
  - Removed `KF2510`; all 6 peripheral ports use vertical THT `JST-XH 2.50 mm` (`2P, 3P, 4P×2, 6P, 8P`).
  - Convention locked: `Pin 1 = Power (+)` and `Pin 2 = GND` on every power-carrying port.

---

## 2. Roadmap & Acceptance Gates (M0 → M5)

| Milestone | Mandatory Acceptance Evidence | Status |
|---|---|---|
| **M0 — Hardware Verification** | Cold checks done (`measured`): `PA9` isolated from `5V/VBUS`, `CSB/NCS ~9 kΩ`, `BN-880 I2C ~18 kΩ`, pad order MPU6500/BMP280/ESC. WeAct F405 V11 official vendor docs (`confirmed_from_vendor_doc`): `HSE 8 MHz` (`X2`), `2×15` Top View `1..30` pinmap & `ΔX = 27.94 mm` (`25.40 mm` inner-to-inner). **Remaining:** print & verify 1:1 A4 physical fit check against real boards, measure `JST-XH` cable end-to-end polarity (`1→1` vs `1→N`), capture `MC7RE` SBUS waveform, stress-test UBEC `5V/3A` under dual-ESP32 load. | **In Progress** |
| **M1 — PCB Rev K & FC Port** | **Firmware (`PASS` / `measured`):** `OPENFCF405` compiled on `Copter-4.6.3` at **`735,816 / 983,040 B`** (`247,224 B` headroom; `AP_Bootloader.bin` `14,080 / 32,768 B`), `OSCILLATOR_HZ 8000000` (`confirmed_from_vendor_doc`), EKF3/PreArm/Failsafes/PrecLand + 3× BN-880 compass drivers ELF-verified, **`TIM4_UP` DMAR dedicated (`SHARED_DMA_MASK = 0`)**. **Schematic (`PASS` / `design`):** 4-sheet KiCad 10 project (`open-fc-carrier.kicad_sch` + `sheet1..3`, `ADR-003` external SBUS inverter -> `R12 4.7 kΩ` series only) with `U1_P1/U1_P2` `verified_from_vendor_doc`, **0 ERC violations**, BOM sync `PASS` (`35` electrical THT parts: 23 discrete + 12 headers/connectors + `4` M3 holes `H1..H4`). **PCB Layout (`PASS` / `design`):** **`60 × 68 mm`** (`Rev K.1`, `ADR-003`) 2-layer FR4 (`F.Cu` routing + continuous `B.Cu` `GND_BCU` plane, `308` tracks, `67` vias, `45 × 45 mm` M3, `25.40 mm` WeAct inner-to-inner, open-sky `U1 33.22×41.60` + `U2 25.10×14.88` + `U3 15.38×11.50` plotted 1:1 on `F.Fab`, low-profile discretes `≤ 5.5 mm` nested under `U1`, `VBAT_RAW` `0.50 mm` clearance netclass), **0 DRC errors, 0 warnings, 0 unconnected, 0 schematic parity issues** (`./scripts/verify-kicad.sh` PASS); **1:1 A4 PDF** exported to `hardware/fabrication/open-fc-carrier-rev-k-1to1-A4.pdf` (physical paper fit check `NOT RUN`). | **In Progress (FW + Sch ERC=0 + PCB DRC=0 + 1:1 A4 PDF PASS; Physical Fit Check Pending)** |
| **M2 — Carrier Bring-up & Gateway** | Staged soldering of Rev K PCB (verify `PA0` voltage before inserting `U1`). Flash ArduCopter, verify IMU/Baro/GPS/Compass/SBUS/Battery Failsafe on bench (props off). Bring up ESP-IDF MAVLink2 Wi-Fi bridge on ESP32 DevKit. | Planned |
| **M3 — Vision Pipeline** | Calibrate `OV2640` (intrinsics/extrinsics), implement marker detection on ESP32-CAM sending MAVLink2 `LANDING_TARGET`, benchmark FPS/latency and stale/bad frame rejection. | Planned |
| **M4 — SITL & Bench Integration** | End-to-end ArduPilot SITL + vision replay. Full F450 bench test (props off) with fault injection (RC loss, Wi-Fi drop, camera occlusion, simulated battery sag). | Planned |
| **M5 — Flight Release** | IMU anti-vibration sign-off (MPU6500 tail locking screw + M3 rubber standoffs), BMP280 wind foam, multimeter calibration of `BATT_VOLT_MULT`, `docs/safety-case.md` approval. | **BLOCKED** |

---

## 3. Decision Log

| Date | Decision | Summary | Record | Status |
|---|---|---|---|---|
| `2026-11-26` | `Carrier PCB Rev K (100% THT)` | Drop `TMUX1511PW`, `Adapter TSSOP14`, `100 kΩ` VBAT pull-down (use `220k/18k + 4.7k + 100nF`); drop external SPI CS pull-ups (`~9 kΩ` measured); add `R9/R10 4.7 kΩ` on `I2C2`; lock Power Option 1 (`UBEC → J_PWR`, star split + `D1 1N5822`); standardize 6 ports on `JST-XH 2.50 mm` (drop `KF2510`). | [001-esc-rev-j-to-rev-k.md](decisions/001-esc-rev-j-to-rev-k.md), [002-power-option1-pixhawk.md](decisions/002-power-option1-pixhawk.md) | `Design Freeze — Ready for KiCad` (`confirmed`) |
| `2026-11-27` | `WeAct F405 V11 Vendor Lock & PCB Rev K Layout` | Locked `OSCILLATOR_HZ = 8000000` (`X2` 8 MHz HSE), `2×15` Top View `1..30` pinmap chirality, and `ΔX = 27.94 mm` (`25.40 mm` inner-to-inner) from official WeActStudio V11 SchDoc/Board Shape/STEP. Upgraded `U1_P1/U1_P2` schematic to `verified_from_vendor_doc`. | [hw-evidence.md](hw-evidence.md), [pinmap-verified.csv](pinmap-verified.csv), `hardware/reference-schematics/weact-f405-v11/` | `Locked` (`confirmed_from_vendor_doc`) |
| `2026-11-27` | `Rev K.1: External SBUS Inverter & 60×68 mm Open-Sky Sensor Layout` | Removed onboard `Q1/R11/R13/JP_SBUS1` (external inverted SBUS carrier feeds `J_RC1.3 → R12 4.7 kΩ → PA1`); expanded outline to `60 × 68 mm` (`45 × 45 mm` M3 preserved) so `MPU6500 (25.10×14.88)` & `BMP280 (15.38×11.50)` sit in 100% open sky (`8.5 mm` sockets, zero Z-collision with WeAct) while low-profile discretes (`≤ 5.5 mm`) nest under WeAct `U1`; `0` ERC / `0` DRC / `0` parity; 1:1 A4 PDF exported with `F.Fab` module outlines. | [003-sbus-ext-inverter-and-sensor-envelope.md](decisions/003-sbus-ext-inverter-and-sensor-envelope.md), [hw-evidence.md](hw-evidence.md) | `DRC=0 / 1:1 A4 Exported — Pending Physical Fit Check` (`design` / `confirmed`) |

---

## 4. Synchronized Artifacts

Any modification to pins, component values, or connectors must update in lockstep:
- `docs/pinmap-verified.csv`
- `docs/bom-rev-k.csv`
- `hardware/kicad/open-fc-carrier-rev-k/*`
- `firmware/board-openfcf405/hwdef.dat`
- `docs/OPEN_FC_MINIMAL_v1.0.md`
