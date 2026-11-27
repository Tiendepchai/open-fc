# ADR-003: External SBUS Inverter & 60 × 68 mm Open-Sky Sensor Layout (Rev K.1)

- **Date:** `2026-11-27`
- **Status:** `confirmed` (`design` — pending physical 1:1 A4 fit check)

## Context

Two physical bench inputs updated the original Rev K (`60 × 56 mm`) assumptions:
1. **Measured breakout dimensions (`docs/hw-evidence.md`):**
   - `MPU6500` (`U2`, `1×10`): **`25.10 × 14.88 mm`**
   - `BMP280` (`U3`, `1×6`): **`15.38 × 11.50 mm`**
   - The inner clear bay between WeAct STM32F405 `U1_P1` and `U1_P2` female socket bodies is `22.86 × 38.10 mm` (`14.88 + 11.50 = 26.38 mm > 22.86 mm`, cannot sit side-by-side inside `U1`), and standard `8.5 mm` `PinSocket`s + module header (`~13.6 mm` total height) collide in Z with WeAct's underside MicroSD slot (`~9.2 mm` clearance).
2. **External inverted SBUS carrier:** The RC SBUS link from `MC7RE` is already inverted by an external SBUS carrier PCB before entering `J_RC1`.

## Decision

1. **Remove onboard discrete SBUS inverter (`Q1 2N3904`, `R11 10 kΩ`, `R13 4.7 kΩ`, `JP_SBUS1 1×3`):**
   - Retain a single **`R12 = 4.7 kΩ`** series protection resistor (`J_RC1.3 → R12 → SBUS_MCU_PA1 / UART4_RX`). On `STM32F405RGT6`, `PA1` is an analog `TTa` (**3.3 V max, non-FT**) pin; `4.7 kΩ` caps ESD-clamp injection current to `~0.30 mA` (`≪ 5 mA` abs max) even on a `5 V` idle-HIGH external SBUS output while adding negligible RC delay (`τ ≈ 47 ns` vs `10 µs` SBUS bit width = `0.5%`).
   - BOM shrinks from `39` to **`35` electrical THT parts** (`23` discrete + `12` headers/connectors) + `4` M3 mounting holes (`H1..H4`).
2. **Adopt `60 × 68 mm` 2-layer FR4 outline (`45 × 45 mm` F450 M3 pattern preserved) with strict 3D Z-tier separation:**
   - **Under-WeAct low-profile bay (`X = 118.6..141.4, Y = 127.0..165.0`, `Z_clear ≥ 9.2 mm`):** Packs all low-profile THT discretes (`14×` horizontal `1/4W` resistors `2.5 mm`, `D1 1N5822` `5.3 mm`, `C3..C5` `5.5 mm` — `≥ 3.7 mm` vertical clearance below WeAct MicroSD slot).
   - **Open-sky perimeter & sensor deck (zero Z-stack conflict, standard `8.5 mm` sockets):**
     - `U1` (`WeAct F405`, `33.22 × 41.60 mm`, USB-C flush at bottom edge for unobstructed cable insertion).
     - `U2` (`MPU6500`, `25.10 × 14.88 mm`, horizontal above `U1`) and `U3` (`BMP280`, `15.38 × 11.50 mm`, vertical beside `U2`) — full open sky for M5 anti-vibration tail support and BMP280 wind foam.
     - All 6× `JST-XH`, `C1/C2 470 µF`, `J_TEST1`, and `JP_C1` (DEFAULT OPEN) in accessible open sky.
   - Exact 1:1 physical outlines of `WeAct F405`, `MPU6500`, and `BMP280` are plotted on `F.Fab` inside the 1:1 A4 fit-check PDF.

## Consequences

- Zero X/Y or Z-height mechanical interference with standard 100% THT `8.5 mm` `PinSocket`s; board stays well inside standard `100 × 100 mm` low-cost 2-layer fab tier.
- `U2 (MPU6500)` sits rotated `90°` CCW on the carrier PCB relative to the earlier draft; verify exact `INS_ORIENT` / `IMU ROTATION_*` in M2 prop-off Mission Planner / MAVProxy orientation check.
- `M0-07` still requires verifying the external SBUS carrier output voltage (`3.3 V` vs `5 V`) and idle-HIGH polarity before connecting `U1`.
