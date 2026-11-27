# ADR-002: Power Architecture Option 1 (Pixhawk Style) & USB-C Backfeed Isolation

- **Date:** `2026-11-26`
- **Status:** `confirmed` (`Design Freeze — Ready for KiCad`)

## Context

The carrier powers both flight-critical logic (`WeAct STM32F405RGT6`, `BN-880` GPS/Compass, `MC7RE` RC receiver) and bursty RF/vision companion loads (`ESP32 DevKit` Wi-Fi + `ESP32-CAM` OV2640). Bench development also connects USB-C to the FC while battery/UBEC power is disconnected, creating a backfeed risk into companion boards and the UBEC output.

## Decision

Adopt **Power Option 1 (Pixhawk style)**:
1. **External UBEC `5V/3A`:** Solder UBEC input leads (20–22 AWG) directly to the Uangel 60A ESC main `BAT+ / BAT-` pads. Terminate UBEC `5V` output into `J_PWR (JST-XH 2P)` (`Pin 1 = +5V_COMP`, `Pin 2 = GND`).
2. **Star Rail Split on Carrier PCB:**
   - **`+5V_COMP`** (pre-`D1`, backed by `C1/C2 470 µF` bulk electrolytics): Feeds high-burst companion ports `J_WIFI` and `J_CAM`.
   - **`+5V_FC`** (post-Schottky `D1: 1N5822`, decoupled by `C6 100 nF`): Feeds `U1 (5V)`, `J_GPS`, and `J_RC`.
3. **High-Current Isolation:** Carrier PCB carries logic/signal currents only. `J_ESC Pin 2 (VBAT_RAW, up to 12.6 V)` feeds ONLY the high-impedance `PA0` resistor divider (`220 kΩ / 18 kΩ`).

## Consequences

- `D1 (1N5822)` prevents USB-C `5V` from backfeeding `+5V_COMP` (ESP32 boards) or the unpowered UBEC, and isolates `+5V_FC` from transient companion current spikes in combination with `C1/C2 470 µF`.
- Forward drop across `1N5822` (`~0.25–0.4 V` at FC+GPS+RC load) must be verified under dual-ESP32 peak load (`M0` / `docs/power-verification.md`) to confirm adequate headroom for the WeAct onboard `3.3 V` LDO.
