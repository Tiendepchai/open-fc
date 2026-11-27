# MAVLink2 Interface Contract (FC ↔ Companion Boards)

> **Status:** `candidate` (UART pinout `confirmed`/`design`; baudrates, stream rates, and timeouts pending M2/M3 validation).
> **Authority Rule (`AGENTS.md §1.1`):** Companions NEVER drive ESCs or bypass FC arming, EKF, or failsafes.

## 1. Serial Port Allocation (`OPENFCF405`)

| ArduPilot Port | STM32 Peripheral | Carrier Port | Peer | Protocol (`candidate`) | Candidate Baud |
|---|---|---|---|---|---|
| `SERIAL0` | `OTG1` (`PA11/PA12`) | WeAct USB-C | Ground Station / Bench USB | `2` (MAVLink2) | USB CDC |
| `SERIAL1` | `USART1` (`PA9/PA10`) | `J_WIFI` (`JST-XH 4P`) | `ESP32 DevKit` (Wi-Fi UDP bridge) | `2` (MAVLink2) | `921600` (`candidate`; fallback `115200`/`460800`) |
| `SERIAL2` | `USART2` (`PA2/PA3`) | `J_CAM` (`JST-XH 4P`) | `ESP32-CAM` (OV2640 vision) | `2` (MAVLink2) | `115200` (`candidate`; low-bandwidth `LANDING_TARGET`) |
| `SERIAL3` | `USART6` (`PC6/PC7`) | `J_GPS` (`JST-XH 6P`) | `BN-880` GPS | `5` (GPS) | `38400` / auto-config |
| `SERIAL4` | `UART4` (`PA1` RX-only) | `J_RC` (`JST-XH 3P`) | `MC7RE` SBUS (via `Q1 2N3904`) | `23` (RCIN) | `100000` (8E2 via `AP_RCProtocol`) |

## 2. Companion Message Contracts (`candidate`)

### 2.1. `esp32-wifi` (`SERIAL1` — MAVLink2 UDP Bridge)
- Transparent bidirectional MAVLink1/MAVLink2 packet forwarder (up to `280 B` signed MAVLink2) between `USART1` and GCS UDP endpoint (`candidate` port `14550`, dynamic GCS peer learning on validated inbound datagram).
- **Framing & Recovery (`design`):**
  - UART→UDP stream framer (`mavlink_framer.c`) rejects unknown MAVLink2 `incompat_flags` (non-`0x01`), rescans embedded STX bytes in rejected headers without losing adjacent frames, and resets stalled partial packets on UART errors or `100 ms` UART idle (`candidate`).
  - UDP→UART trust boundary (`openfc_mavlink_validate_datagram`) accepts single or coalesced multi-packet MAVLink1/2 datagrams up to `1500 B` (Ethernet IPv4 MTU headroom) and rejects truncated, malformed, or garbage datagrams before touching FC UART.
  - Non-blocking `sendto(..., MSG_DONTWAIT)`, synchronized dual-core peer/socket state, automatic UDP socket recovery on fatal socket errors, and Wi-Fi STA reconnect handling without blocking UART RX or injecting synthetic commands.
- Wi-Fi credentials live only in untracked local config; tracked `sdkconfig.defaults` uses placeholders only and firmware never logs credentials (`AGENTS.md §1.4`).

### 2.2. `esp32-cam` (`SERIAL2` — Vision Precision Landing)
- **Identity (`candidate`):** `sys_id = 1` (matches FC), `comp_id = 191` (`MAV_COMP_ID_ONBOARD_COMPUTER`).
- **Outbound to FC:**
  - `HEARTBEAT` (`#0`, `MAV_TYPE_ONBOARD_CONTROLLER`, `1 Hz`) for link liveness monitoring (`MAV_STATE_STANDBY` when camera init fails or intrinsics are uncalibrated; `MAV_STATE_ACTIVE` when operational and calibrated).
  - `LANDING_TARGET` (`#149`, `LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2`) on valid marker detections only:
    - `frame`: `MAV_FRAME_BODY_FRD` (`12`; confirm camera-to-body rotation in `docs/camera-calibration.md`).
    - `angle_x`, `angle_y`: angular offset (radians) derived from calibrated intrinsics matching `AC_PrecLand_Companion.cpp:45`.
    - `position_valid = 0` (`candidate`, pure angular LOS; `71 B` MAVLink2 wire frame after trailing-zero truncation, `q = [1, 0, 0, 0]`). Wire packer refuses non-finite values, negative size/distance, or unsupported `position_valid != 0` until 3D pose is calibrated and validated in M3.
- **Safety / Fault Handling (`design`):**
  - Refuses to emit `LANDING_TARGET` when intrinsics are uncalibrated (`fx = 0` or `fy = 0`), on framebuffer format/dimension/length mismatch, on occluded/blurry/low-confidence/streak/noise frames, on stale/out-of-order frames (`age > 150 ms` `candidate`), or on single-frame outlier jumps (`> 0.45 rad` within `300 ms` `candidate`).
  - Re-acquires valid targets automatically after sustained occlusion (`dt > outlier_window_us`, wrap-safe `uint64_t` arithmetic) without permanent outlier lockout.
  - On vision loss, FC `PLND_STRICT` / `LAND` behavior remains solely under ArduCopter authority.
