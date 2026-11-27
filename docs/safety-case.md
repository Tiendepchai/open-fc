# Safety Case & M5 Flight Release Gate

> **Flight Release Status:** **`BLOCKED`** — Hardware design (Rev K) and M0 verification in progress. **BENCH ONLY (PROPELLERS REMOVED).**

## 1. Non-Negotiable Safety Invariants

1. `WeAct STM32F405RGT6` (`OPENFCF405` / ArduCopter) is the sole flight-critical controller.
2. Companions (`ESP32 DevKit`, `ESP32-CAM`) have no electrical path to ESC signals and cannot bypass arming, EKF, or failsafes.
3. All bench testing (M2–M4) MUST be conducted with **all propellers physically removed**.
4. Compilation pass, KiCad ERC/DRC pass, or SITL pass NEVER constitutes flight readiness.

## 2. Mandatory Pre-Arm & Failsafe Matrix (`candidate` → verify in M2/M4)

| Fault Condition | Expected FC Action | Bench Fault-Injection Test (M4) | Verdict |
|---|---|---|---|
| Pre-arm failure (EKF, compass, baro, RC, battery) | Arming denied (`ARMING_CHECK = 1`) | Verify each pre-arm check blocks arming | `NOT RUN` |
| RC link loss (`MC7RE` power off / TX off) | `FS_THR_ENABLE` → RTL / Land | Power off RC transmitter on bench; observe mode transition | `NOT RUN` |
| Battery low / critical (`PA0` calibrated) | `BATT_FS_LOW_ACT` → RTL; `BATT_FS_CRT_ACT` → Land | Bench power supply voltage sweep below thresholds | `NOT RUN` |
| Wi-Fi telemetry drop (`ESP32 DevKit` unplugged) | Flight unaffected; FC continues normally | Disconnect `J_WIFI` mid-run; confirm zero FC glitch | `NOT RUN` |
| Vision loss / occluded marker (`ESP32-CAM`) | Precision landing falls back cleanly to normal Land/pilot control | Cover lens / unplug `J_CAM`; confirm no stale `LANDING_TARGET` | `NOT RUN` |

## 3. M5 Unblock Checklist (All Must Be `validated`)

- [ ] M0 complete (`JST-XH` cable polarity, `MC7RE` SBUS waveform, UBEC `5V/3A` dual-ESP32 load test).
- [ ] M1 complete (Rev K ERC/DRC pass, 1:1 A4 fit pass, `OPENFCF405` Flash `< 960 KB` app slot, `TIM4` DMA verified).
- [ ] M2 complete (staged Rev K bring-up, `PA0` pre-MCU check, all sensors & failsafes verified prop-off).
- [ ] M3 complete (`OV2640` calibration, `LANDING_TARGET` sign & latency verified).
- [ ] M4 complete (SITL + vision replay + F450 prop-off fault-injection logs archived in `reports/`).
- [ ] Mechanical IMU isolation complete (`MPU6500` tail locking screw + M3 rubber standoffs).
- [ ] `BMP280` open-cell wind foam installed and baro noise checked.
- [ ] `BATT_VOLT_MULT` calibrated against physical DMM and logged in `docs/power-verification.md`.
- [ ] Explicit human sign-off recorded below before installing propellers.

**Sign-off:** `BLOCKED — NOT APPROVED FOR FLIGHT`
