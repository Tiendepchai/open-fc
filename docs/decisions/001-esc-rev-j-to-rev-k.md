# ADR-001: Carrier PCB Rev J → Rev K (100% THT, Passive VBAT Protection, Standardized JST-XH)

- **Date:** `2026-11-26`
- **Status:** `confirmed` (`Design Freeze — Ready for KiCad`)

## Context

Earlier carrier iterations included an SMD analog switch (`TMUX1511PW` on a `TSSOP14` adapter) with a `100 kΩ` pull-down for VBAT backfeed isolation, mixed connector footprints (`KF2510` alongside JST), and external SPI CS pull-up resistors. Cold bench measurements on the actual hardware (`M0`) changed several assumptions:
- `MPU6500 (NCS)` and `BMP280 (CSB)` already contain `~9 kΩ` internal pull-ups (`measured`).
- `BN-880` has `~18 kΩ` pull-ups on `SDA/SCL` (`measured`), which is too weak for clean `400 kHz` I²C over a GPS mast cable.
- `PA9` on `WeAct STM32F405RGT6` is isolated from `5V/VBUS` (`measured`).

## Decision

Lock **Carrier PCB Rev K** (`60 × 56 mm`, 2-layer FR4, 4× M3 `45 × 45 mm`) to **100% THT** (target 26 discrete components + 9 headers/sockets):
1. **Drop `TMUX1511PW`, `Adapter TSSOP14`, and `100 kΩ` VBAT pull-down.** Replace with a passive current-limited divider `220 kΩ (1%) / 18 kΩ (1%)` + `4.7 kΩ` series + `100 nF` into `PA0` (`ADC1_IN0`). Unpowered FC leakage at 3S `12.6 V` is `~33 µA` (well within STM32 ESD clamp limits).
2. **Keep `PB6–PB9` ESC conditioning** (`47 Ω` series `R1–R4`, `10 kΩ` pull-down `R5–R8`) and leave `JP_C` (`PC0` current sense + `R15 10 kΩ`) **DEFAULT OPEN**.
3. **Remove external SPI CS pull-ups** for `MPU6500` (`PA4`) and `BMP280` (`PB12`).
4. **Add `R9/R10 4.7 kΩ` I²C2 pull-ups** to `+3V3` on the Carrier PCB for `BN-880` (`PB10/PB11`).
5. **Drop `KF2510` and standardize all 6 peripheral ports on vertical THT `JST-XH 2.50 mm`** (`2P, 3P, 4P×2, 6P, 8P`) with `Pin 1 = Power (+)` and `Pin 2 = GND` on every power-carrying port.

## Consequences

- Eliminates all SMD/adapter soldering complexity and failure points; PCB is directly hand-solderable and ready for bench validation after ERC/DRC and 1:1 A4 fit verification.
- Nominal `BATT_VOLT_MULT` becomes `(220 + 18) / 18 = 13.22` (`design`; requires DMM calibration at M2/M5).
- Superseded parts (`TMUX1511PW`, `Adapter TSSOP14`, `100 kΩ`, `KF2510`, `RuiBet 55A`, `TIM8 PC6–PC9`, external SPI CS pull-ups) must NOT be reintroduced without a new ADR.
