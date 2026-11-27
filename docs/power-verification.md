# Power Architecture Verification (Option 1 — Pixhawk Style)

> **Baseline:** External UBEC `5V/3A` wired directly to Uangel 60A ESC `BAT+ / BAT-` (20–22 AWG) → `J_PWR (JST-XH 2P)` → star split `+5V_COMP` (`C1/C2 470 µF`) and `+5V_FC` via Schottky `D1 1N5822` (`C6 100 nF`).
> **Status:** `design` (bench measurements `NOT RUN`).

## 1. Pre-MCU Bring-up Gate (Mandatory Before Inserting `U1`)

Do NOT insert `WeAct STM32F405RGT6` into `U1` socket until ALL rows below record `PASS`:

| Check | Setup | Expected (`design`) | Measured | Verdict |
|---|---|---|---|---|
| Continuity short check | Unpowered PCB, DMM resistance | No short between `VBAT_RAW`, `+5V_COMP`, `+5V_FC`, `+3V3`, `GND` | `TBD` | `NOT RUN` |
| `VBAT_RAW` isolation | 3S bench supply (`11.1–12.6 V`) on `J_ESC Pin 2` | `PA0` pad reads `V_bat / 13.22` (`<= 0.96 V` at `12.6 V`); `0 V` on `+5V_FC` and `+3V3` | `TBD` | `NOT RUN` |
| `JP_C` open verification | `J_ESC Pin 7` floating/powered | `JP_C` open; `PC0` isolated from uncharacterized `CURR_RAW` | `TBD` | `NOT RUN` |
| `+5V_COMP` rail (no load) | UBEC powered via `J_PWR` | `~5.00 V` (±3%) | `TBD` | `NOT RUN` |
| `+5V_FC` rail (post `D1 1N5822`) | UBEC powered via `J_PWR` | `~4.6–4.8 V` (Schottky drop across `1N5822`), within WeAct 5V LDO headroom | `TBD` | `NOT RUN` |
| USB-C backfeed block | Power `U1` via USB-C only (`J_PWR` unplugged) | `+5V_COMP` remains `~0 V`; `D1` blocks reverse current into companions/UBEC | `TBD` | `NOT RUN` |

## 2. Dual-ESP32 Dynamic Load Stress Test (M0 / M2)

| Load Profile | `+5V_COMP` Min (V) | `+5V_FC` Min (V) | UBEC Temp (°C) | FC Brownout/Reset? | Verdict |
|---|---|---|---|---|---|
| Idle (`U1` + `BN-880` + `MC7RE`) | `TBD` | `TBD` | `TBD` | `TBD` | `NOT RUN` |
| `ESP32 DevKit` Wi-Fi TX + `ESP32-CAM` active streaming/capture | `TBD` | `TBD` | `TBD` | `TBD` | `NOT RUN` |

## 3. Battery Voltage Calibration (`BATT_VOLT_MULT`)

- **Design nominal (`design`):** `(220 kΩ + 18 kΩ) / 18 kΩ = 13.222` (`BATT_VOLT_MULT = 13.22`).
- **Calibration procedure (M2 & M5):** Measure actual 3S LiPo terminal voltage with calibrated DMM (`V_dmm`) and adjust `BATT_VOLT_MULT` until MAVLink `SYS_STATUS.voltage_battery` matches within `±0.05 V` across 3S working range (`10.5 V – 12.6 V`).
- **Measured value:** `TBD` (`NOT RUN`).
