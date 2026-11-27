# Camera Calibration & Vision Frame Convention (`OV2640` / `ESP32-CAM`)

> **Milestone:** `M3 — Vision Pipeline` (`Planned`)
> **Status:** `TBD` — No intrinsics, FPS, or accuracy numbers fabricated prior to physical capture.

## 1. Hardware Setup (`design`)

- **Module:** `ESP32-CAM` with `OV2640` sensor powered from `J_CAM` (`+5V_COMP`).
- **Resolution & Pixel Format (`candidate`):** `TBD` (benchmark grayscale QVGA/QQVGA vs latency in M3).
- **Lens:** `TBD` (confirm standard FOV vs wide-angle OV2640 module mounted on F450).

## 2. Intrinsics (`NOT RUN`)

Record after checkerboard / ChArUco calibration:
- `fx`, `fy`, `cx`, `cy`: `TBD`
- Distortion coefficients `(k1, k2, p1, p2, k3)`: `TBD`
- Reprojection error (px): `TBD`
- Calibration dataset path: `sim/vision-replay/`

## 3. Extrinsics & Frame Convention (`confirmed` against `Copter-4.6.3`)

- **Body frame (ArduPilot FRD):** `+X` Forward, `+Y` Right, `+Z` Down.
- **ArduCopter `AC_PrecLand_Companion.cpp:45` angular LOS convention (`position_valid = 0`):**
  ```cpp
  _los_meas_body = Vector3f(-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f);
  ```
  - Image top (`v < cy`) = Forward (`+X` FRD) → `angle_y < 0` → `-tan(angle_y) > 0`.
  - Image right (`u > cx`) = Right (`+Y` FRD) → `angle_x > 0` → `+tan(angle_x) > 0`.
  - Camera FRD ray at `z_cam = +1` (downward optical axis):
    - `cam_fwd   = -(v - cy) / fy` (`+X_cam`, image top)
    - `cam_right = +(u - cx) / fx` (`+Y_cam`, image right)
  - If mounted rotated by yaw `theta` (`cam_yaw_offset_rad`, CW about `+Z` Down from body `+X` to camera `+X`, matching ArduCopter `AC_PrecLand.cpp:622` `rotate_xy`):
    - `body_fwd   = cos(theta) * cam_fwd - sin(theta) * cam_right`
    - `body_right = sin(theta) * cam_fwd + cos(theta) * cam_right`
    - `angle_x = atan2f(body_right, 1.0f)`, `angle_y = atan2f(-body_fwd, 1.0f)`
  - **Double-rotation rule (`design`):** Apply camera yaw offset in EITHER `esp32-cam` (`CONFIG_OPENFC_CAM_YAW_OFFSET_MDEG`) OR ArduCopter (`PLND_YAW_ALIGN`), never both.
  - ArduCopter marks LOS stale after `1000 ms` (`AC_PrecLand_Companion.cpp:19`); companion enforces stricter `max_frame_age_ms` (`candidate: 150 ms`) with `uint64_t` wrap-safe age/sequence checks and automatic outlier-filter re-acquisition after sustained occlusion (`> outlier_window_us`, `candidate: 300 ms`).
  - **Fail-closed calibration gate (`design`):** `CONFIG_OPENFC_CAM_FX_MILLIPX=0` or `FY=0` (tracked defaults) blocks `LANDING_TARGET` TX while maintaining 1 Hz `HEARTBEAT` (`MAV_STATE_STANDBY`).
- **Camera-to-body mounting offset `(dx, dy, dz)` (m):** `TBD` (measure on physical F450 mount).
- **Yaw alignment (`cam_yaw_offset_deg`):** `TBD` (verify sign of `angle_x`, `angle_y` in bench tilt/shift test before M4).

## 4. Marker & Benchmark Evidence (`NOT RUN`)

- **Marker family & physical edge size (mm):** `TBD` (e.g., AprilTag / ArUco — select in M3).
- **Measured FPS / end-to-end latency / drop rate:** `NOT RUN` (log in `reports/`).
