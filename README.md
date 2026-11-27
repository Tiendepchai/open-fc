# open-fc (OPEN-FC MINIMAL v1.0 — Carrier PCB Rev K)

> **Status:** `HARDWARE DESIGN (REV K) & M0 VERIFICATION — BENCH ONLY` (Not flight-certified).

Custom minimal flight-controller carrier & companion stack built around a `WeAct STM32F405RGT6` running `ArduCopter / ChibiOS` (`OPENFCF405`) as the single flight-critical authority, paired with `ESP32 DevKit` (MAVLink2 Wi-Fi telemetry bridge) and `ESP32-CAM` (`OV2640` vision precision landing).

## Baseline Summary (Rev K — 100% THT)

- **Carrier PCB (`design`):** `60 × 68 mm` (`Rev K.1`, ADR-003), 2-layer FR4, 4× M3 mounting holes (`45 × 45 mm`), 100% THT (`23` discrete + `12` headers/sockets = `35` electrical parts + `4` M3 holes). Open-sky `U1` (`33.22×41.60 mm`), `U2` (`25.10×14.88 mm`), `U3` (`15.38×11.50 mm`); low-profile discretes (`≤ 5.5 mm`) nested under `U1`.
- **VBAT Sense (`design`):** Passive current-limited divider `220 kΩ (1%) / 18 kΩ (1%)` + `4.7 kΩ` series + `100 nF` into `PA0` (`TMUX1511PW` removed). `JP_C` (`PC0` current sense) stays **OPEN** by default.
- **Power Option 1 — Pixhawk Style (`confirmed`):** External UBEC `5V/3A` off ESC `BAT+ / BAT-` into `J_PWR (JST-XH 2P)`, star-routed into `+5V_COMP` (`J_WIFI`, `J_CAM`, `C1/C2 470 µF`) and `+5V_FC` via Schottky `D1 1N5822` (`U1`, `J_GPS`, `J_RC`, `C6 100 nF`).
- **Sensors & Peripherals (`measured` / `design`):** `MPU6500` + `BMP280` on `SPI1` (internal `~9 kΩ` CS pull-ups measured; external CS pull-ups removed), `BN-880` on `USART6` + `I2C2` (`18 kΩ` measured on module; PCB adds `R9/R10 4.7 kΩ` to `+3V3`), `MC7RE` SBUS on `UART4` via external inverted SBUS carrier PCB + `R12 (4.7 kΩ)` series protection (ADR-003).
- **Connectors (`confirmed`):** Standardized vertical THT `JST-XH 2.50 mm` (`2P, 3P, 4P×2, 6P, 8P`), `Pin 1 = +V`, `Pin 2 = GND` on all power-carrying ports.

## Roadmap Status

| Milestone | Scope | Status |
|---|---|---|
| **M0 — Hardware Verification** | Cold checks done (`PA9`, `NCS/CSB`, `BN-880 I2C`, pad orders); WeAct F405 V11 `8 MHz` HSE + `2×15` pinmap `1..30` + `ΔX = 27.94 mm` locked from vendor docs. Pending: 1:1 A4 paper fit check, `JST-XH` cable wiring polarity, `MC7RE` SBUS waveform, UBEC `5V/3A` dual-ESP32 load test. | **In Progress** |
| **M1 — PCB Rev K & FC Port** | **FW (`PASS`):** `OPENFCF405` (`Copter-4.6.3`, `HSE 8 MHz`) builds at `735,816 / 983,040 B` (`247 KB` headroom) with `SHARED_DMA_MASK=0` (`TIM4_UP` DMAR dedicated). **Schematic (`PASS`):** 4-sheet KiCad 10 Rev K.1 (`U1_P1/U1_P2` `verified_from_vendor_doc`, external SBUS carrier + `R12 4.7 kΩ`, `0` ERC violations, BOM sync `PASS` `23` discrete + `12` headers/connectors). **PCB (`PASS` / `design`):** `60 × 68 mm` 2-layer layout (`B.Cu` GND plane, `308` tracks, `67` vias, `45 × 45 mm` M3, `25.40 mm` WeAct inner-to-inner, open-sky `MPU6500`/`BMP280`/`WeAct` with 1:1 `F.Fab` outlines, low-profile discretes under `U1`), **`0` DRC errors/warnings/unconnected/parity**; **1:1 A4 PDF** at `hardware/fabrication/open-fc-carrier-rev-k-1to1-A4.pdf` (physical fit check pending). | **In Progress (FW + Sch + PCB DRC=0 PASS; Fit Check Pending)** |
| **M2 — Carrier Bring-up & Gateway** | Staged PCB solder (`PA0` check pre-MCU), ArduCopter bench bring-up (props off), `esp32-wifi` MAVLink2 UDP↔UART1 bridge. | Planned |
| **M3 — Vision Pipeline** | `OV2640` calibration, `esp32-cam` marker detection → MAVLink2 `LANDING_TARGET`, FPS/latency benchmark. | Planned |
| **M4 — SITL & Bench Integration** | ArduCopter SITL + vision replay; F450 prop-off bench fault-injection tests. | Planned |
| **M5 — Flight Release** | IMU damping, BMP280 wind foam, measured `BATT_VOLT_MULT`, `docs/safety-case.md` approval. | **BLOCKED** |

## Repository Map

- `AGENTS.md` — mandatory engineering guardrails and safety rules.
- `docs/` — master baseline (`OPEN_FC_MINIMAL_v1.0.md`), verified pinmap, Rev K BOM, ADRs, verification logs, safety case.
- `hardware/` — KiCad Rev K project, reference schematics, fabrication outputs, mechanical mounts.
- `firmware/` — pinned `ardupilot/` submodule, custom `board-openfcf405/`, `esp32-wifi/`, `esp32-cam/`.
- `sim/` — SITL launch configs, vision replay datasets, fault-injection scenarios.
- `scripts/` — FC build + 1 MB Flash check (`build-fc.sh`), ChibiOS DMA analyzer (`check-dma.py`), BOM sync check (`check-bom-sync.py`).
- `reports/` — dated build logs, ERC/DRC reports, bench verification outputs.

## Quick Commands (M1–M4 Software Verification)

```bash
# 1. Verify BOM CSV consistency (and cross-check KiCad export when available)
python3 scripts/check-bom-sync.py

# 2. Build OPENFCF405 bootloader + ArduCopter (verifies < 1 MB Flash, TIM4 DMA, 11 ELF safety symbols)
./scripts/build-fc.sh

# 3. Verify companion C11 modules (esp32-wifi, esp32-cam), vision replay, and M4 fault-injection suite
./scripts/test-companions.sh

# 4. Run KiCad 10 ERC=0, DRC=0 + schematic parity, 1:1 A4 PDF export, and BOM sync (once PCB exists)
./scripts/verify-kicad.sh
```
