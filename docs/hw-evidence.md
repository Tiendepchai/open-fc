# Hardware Verification Evidence Log (M0)

> **Milestone:** `M0 — Hardware Verification` (`In Progress`)
> **Policy:** Only measurements backed by multimeter/oscilloscope/physical inspection are tagged `measured`. Never extrapolate or fabricate values.

## 1. Completed Cold Checks (`measured`)

| ID | Item | Observed Measurement | Impact on Rev K Design | Status |
|---|---|---|---|---|
| `M0-01` | WeAct STM32F405RGT6 `PA9` isolation | `PA9` isolated from `5V` / `VBUS` | Safe to assign `PA9/PA10` (`USART1`) to `J_WIFI` without USB VBUS conflict | `measured` |
| `M0-02` | `MPU6500` module `NCS` pull-up | `~9 kΩ` internal pull-up measured | Removed external `MPU_CS` pull-up resistor in Rev K | `measured` |
| `M0-03` | `BMP280` module `CSB` pull-up | `~9 kΩ` internal pull-up measured | Removed external `BARO_CS` pull-up resistor in Rev K | `measured` |
| `M0-04` | `BN-880` `SDA/SCL` pull-up | `~18 kΩ` measured on module | Added `R9/R10 4.7 kΩ` pull-ups to `+3V3` on Carrier PCB for `400 kHz` I²C | `measured` |
| `M0-05` | Physical pad order (`MPU6500`, `BMP280`, `Uangel 60A ESC`) | Verified physically against carrier layout | Baseline locked; sync exact pin-by-pin CSV when reference PNGs are committed | `measured` |

---

## 2. Remaining M0 Bench Worksheets (`NOT RUN`)

### M0-06 — `JST-XH` 2-End Harness Polarity (`1→1` vs `1→N`)
- **Goal:** Prevent reversed `+5V` / `GND` or swapped `TX/RX` before locking PCB silkscreen and schematic pins.
- **Method:** Hold both plug housings key-up facing you; beep continuity from End-A Pin 1 to End-B Pin 1 vs Pin N.

| Harness | Port | Pin Count | Measured Wiring (`1→1` or `1→N`) | Red/Black Wire Color Note | Verdict |
|---|---|---|---|---|---|
| UBEC power cable | `J_PWR` | 2P | `TBD` | `TBD` | `NOT RUN` |
| RC cable (`MC7RE`) | `J_RC` | 3P | `TBD` | `TBD` | `NOT RUN` |
| Wi-Fi cable (`ESP32 DevKit`) | `J_WIFI` | 4P | `TBD` | `TBD` | `NOT RUN` |
| Vision cable (`ESP32-CAM`) | `J_CAM` | 4P | `TBD` | `TBD` | `NOT RUN` |
| GPS+Compass cable (`BN-880`) | `J_GPS` | 6P | `TBD` | `TBD` | `NOT RUN` |
| ESC harness (`Uangel 60A`) | `J_ESC` | 8P | `TBD` | `TBD` | `NOT RUN` |

### M0-07 — External SBUS Carrier Output Verification (`ADR-003`)
- **Goal:** Confirm voltage level (`3.3 V` vs `5 V`), already-inverted idle-HIGH UART polarity, frame rate, and failsafe flag behavior from the external inverted SBUS carrier PCB feeding `J_RC1.3 → R12 (4.7 kΩ) → PA1 (UART4_RX)`.
- **Pin Tolerance Note (`STM32F405RGT6`):** `PA1` (`UART4_RX` / `ADC1_IN1`) is an analog **`TTa` (3.3 V max, NOT 5V-tolerant `FT`)** pin. Keeping series `R12 = 4.7 kΩ` limits clamp-diode injection current to **`~0.30 mA`** (`(5.0 - 3.6) V / 4.7 kΩ`, well within STM32F405 `I_INJ(PIN)` `±5 mA` absolute max) even if the external SBUS carrier idles at `5 V`, while still verifying actual idle-HIGH voltage (`3.3 V` vs `5 V`) on scope before plugging `U1`.
- **Method:** Power `MC7RE` + external SBUS inverter carrier from `5V` bench supply (`GND` common with scope). Probe carrier SBUS output pin (`10×` probe, DC coupling, `200 µs/div` and `5 ms/div`).
- **Record:**
  - `V_high` (V): `TBD`
  - `V_low` (V): `TBD`
  - Idle state (`LOW` = inverted SBUS standard / `HIGH` = non-inverted): `TBD`
  - Measured bit width (`~10 µs` = `100 kbps`): `TBD`
  - Frame interval (`ms`) & TX-off failsafe behavior: `TBD`
  - Scope capture saved under `reports/`: `TBD`

### M0-08 — UBEC `5V/3A` Dual-ESP32 Stress Test
- **Goal:** Confirm UBEC `5V/3A` sustains simultaneous `ESP32 DevKit` Wi-Fi TX bursts + `ESP32-CAM` OV2640 active capture without dipping `+5V_COMP` or `+5V_FC` (post `D1 1N5822`) into brownout.
- **Method:** Power UBEC from 3S bench supply (`11.1 V` and `10.5 V` low-pack edge). Connect both ESP32 boards active on `+5V_COMP` + `C1/C2 470 µF` test load, monitor min/avg voltage on oscilloscope (AC/DC) and UBEC case temperature for 10 min.
- **Record:** Log numbers in `docs/power-verification.md` §2.

### M0-09 — Mechanical & MCU Verification for M1 KiCad / `hwdef.dat` (`confirmed_from_vendor_doc` — verify on physical board before soldering)
- **Vendor Source:** `WeActStudio/WeActStudio.STM32F4_64Pin_CoreBoard` @ `42cdd4d` (`Hardware/WeAct-STM32F4_64PIN-CoreBoard_V11 SchDoc.pdf`, `WeAct-STM32F4_64PIN-CoreBoard_V11 Board Shape 外形.pdf`, `WeAct-STM32F4_64PIN-CoreBoard_V11 3D.step`; PDFs archived under `hardware/reference-schematics/weact-f405-v11/`).
- **WeAct STM32F405RGT6 HSE crystal (`OSCILLATOR_HZ`):** **`8000000` (`8 MHz ±10 ppm, 10 pF`, `X2 XTAL3225`, `C7/C16 10 pF`)** — `confirmed_from_vendor_doc` (locked in `firmware/board-openfcf405/hwdef.dat` & `hwdef-bl.dat`; LSE `X1 = 32.768 kHz ±10 ppm`).
- **WeAct STM32F405RGT6 board outline & `2×15` header spacing (`confirmed_from_vendor_doc`):**
  - Board outline: **`33.22 mm (W) × 41.60 mm (H)`**
  - Header rows: two continuous `2×15` (`2.54 mm` pitch, `P1+P3` left and `P2+P4` right; STEP `Y = 53.09..88.65 mm`, 14 steps × `2.54 mm = 35.56 mm`)
  - **Inner-column to inner-column spacing:** **`25.40 mm`** (`10 × 2.54 mm`, STEP `128.78 - 103.38 mm`; edge to inner col `3.91 mm` each side -> `3.91 + 25.40 + 3.91 = 33.22 mm`)
  - **Center-to-center / Pin-1-to-Pin-1 spacing (`ΔX` between `U1_P1` & `U1_P2` on PCB):** **`27.94 mm`** (`11 × 2.54 mm`, STEP `130.05 - 102.11 mm`; outer-to-outer `30.48 mm` = `12 × 2.54 mm`)
  - **KiCad `PinSocket_2x15_P2.54mm_Vertical` chirality (Top View, WeAct component-side UP, USB-C at bottom, `rot=0°`):** KiCad footprint places Odd pads (`1,3..29`) at `X=0` (RIGHT column) and Even pads (`2,4..30`) at `X=-2.54` (LEFT column), Row 1 at TOP (`Y=0`, away from USB-C), Row 15 at BOTTOM (`Y=+35.56`, USB-C end). Therefore WeAct LEFT column (`P1/P3` odd, `P2/P4` odd) maps to KiCad **EVEN** pads `2,4..30`, and WeAct RIGHT column (`P1/P3` even, `P2/P4` even) maps to KiCad **ODD** pads `1,3..29`.
- **WeAct STM32F405RGT6 onboard peripherals (`confirmed_from_vendor_doc`):** Blue LED `D1` on `PB2` (active-HIGH via `5.1 kΩ`, `R4 10 kΩ` pull-down); User KEY `K2` on `PC13`; `R7=0Ω/C14=NC` (`PB9` routed to header), `R8=0Ω/C15=NC` (`PB11` routed to header); `PA9` has no VBUS connection; MicroSD slot `U4` (`PC8-PC12, PD2, PA8`) on bottom side between `P1` and `P2`.
- **Physical 1:1 A4 fit-check checklist (mandatory before fab/soldering):**
  1. Print [open-fc-carrier-rev-k-1to1-A4.pdf](../hardware/fabrication/open-fc-carrier-rev-k-1to1-A4.pdf) at **100% / Actual Size** (disable *Fit / Shrink to printable area*). Verify outline **`60.00 × 68.00 mm`** and M3 hole centers **`45.00 × 45.00 mm`** with caliper/ruler BEFORE overlaying hardware.
  2. Overlay real `WeAct STM32F405` (`ΔX = 27.94 mm` / `25.40 mm` inner, USB-C at bottom, `8 MHz` crystal marking), `MPU6500 (25.10×14.88 mm)`, and `BMP280 (15.38×11.50 mm)` against the 1:1 `F.Fab` outlines to verify pin-1 polarity and body overhang direction.
  3. Test-fit mated `JST-XH` plug housings + wire bend radius (`~6–8 mm` clearance) near `U2/U3` and `C1/C2 470 µF` so harnesses do not rub `BMP280` wind foam or `MPU6500` anti-vibration mount.
- **Sensor breakout caliper dimensions (`measured`, 2026-11-27):**
  - `MPU6500` (`U2`, `1×10` header): **`25.10 × 14.88 mm`** (long edge along `1×10` `22.86 mm` pin span; perpendicular body `14.88 mm`).
  - `BMP280` (`U3`, `1×6` header): **`15.38 × 11.50 mm`** (long edge along `1×6` `12.70 mm` pin span; perpendicular body `11.50 mm`).
  - Mechanical resolution (`ADR-003`, Rev K.1 `60 × 68 mm`): moved `U2 (25.10 × 14.88 mm)` and `U3 (15.38 × 11.50 mm)` into 100% open sky above `U1` (`standard 8.5 mm PinSocket`, zero Z-collision with WeAct bottom MicroSD `~9.2 mm` clearance); all low-profile discretes (`≤ 5.5 mm`) nested cleanly inside `U1`'s underside bay; 1:1 module envelopes plotted on `F.Fab`.

## 3. Evidence Artifacts

Store bench photos, VOM tables, and scope captures under `reports/` and link them here with dates.
