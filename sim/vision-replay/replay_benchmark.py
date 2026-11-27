#!/usr/bin/env python3
"""Deterministic OV2640 Vision Replay, Stress, Yaw-Offset & Closed-Loop Descent Simulation (M3/M4).

Mirrors the exact C logic in firmware/esp32-cam/main/{marker_detector,landing_target_math,mavlink2_tx}.c:
  - Adaptive dark-marker detection on synthetic 8-bit grayscale frames
  - Exact ArduCopter AC_PrecLand_Companion.cpp:45 FRD angular LOS projection:
      _los_meas_body = Vector3f(-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f)
  - Camera yaw-offset rotation verification across 0, 90, 180, 270 deg
  - Synthetic stress cases: salt-and-pepper noise, near-border off-center markers,
    thin streak false-marker rejection, full-frame darkness/brightness, and target
    re-acquisition after sustained occlusion
  - Closed-loop 2D descent convergence simulation across all 4 yaw offsets
  - Real pymavlink MAVLink2 LANDING_TARGET (#149) encoding, golden-vector parity & wire parse
  - Optional live UDP streaming into ArduCopter SITL (--udp host:port)

NOTE (AGENTS.md §3.2): All timing figures emitted by this script are HOST PYTHON SIMULATION
timings ONLY and MUST NEVER be reported as ESP32-CAM hardware FPS.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import IntEnum
import json
import math
from pathlib import Path
import random
import socket
import sys
import time

from pymavlink.dialects.v20 import ardupilotmega as mav


# Golden wire bytes from firmware/esp32-cam/test_host/test_vision_pipeline.c
GOLDEN_MAVLINK2_HEARTBEAT = bytes([
    0xFD, 0x09, 0x00, 0x00, 0x00, 0x01, 0xBF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x12, 0x08, 0x00, 0x04, 0x03, 0xAE, 0xC6,
])
GOLDEN_MAVLINK2_LANDING_TARGET = bytes([
    0xFD, 0x3B, 0x00, 0x00, 0x00, 0x01, 0xBF, 0x95, 0x00, 0x00,
    0x4E, 0x61, 0xBC, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCC,
    0x4C, 0x3D, 0x8F, 0xC2, 0xF5, 0xBC, 0x00, 0x00, 0x20, 0x40,
    0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x0C,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xD7, 0x27,
])
GOLDEN_MAVLINK2_LANDING_TARGET_SEQ255 = bytes([
    0xFD, 0x3B, 0x00, 0x00, 0xFF, 0x01, 0xBF, 0x95, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x3E, 0xB9,
])

U64_MASK = (1 << 64) - 1
HALF_U64 = (1 << 63) - 1


class LTStatus(IntEnum):
    """Exact mirror of openfc_lt_status_t in landing_target_math.h."""

    OK = 0
    ERR_UNCALIBRATED = 1
    ERR_NOT_DETECTED = 2
    ERR_OUT_OF_BOUNDS = 3
    ERR_LOW_CONFIDENCE = 4
    ERR_STALE_FRAME = 5
    ERR_OUTLIER_JUMP = 6
    ERR_NON_FINITE = 7


@dataclass(frozen=True)
class Calib:
    """Exact mirror of openfc_cam_calib_t in landing_target_math.h."""

    width: int = 320
    height: int = 240
    fx: float = 250.0
    fy: float = 250.0
    cx: float = 160.0
    cy: float = 120.0
    yaw_offset_rad: float = 0.0
    min_confidence: float = 0.18
    max_frame_age_us: int = 150_000
    max_step_rad: float = 0.35
    outlier_window_us: int = 300_000


@dataclass
class FilterState:
    """Exact mirror of openfc_lt_filter_state_t in landing_target_math.h."""

    has_prev: bool = False
    prev_capture_us: int = 0
    prev_angle_x: float = 0.0
    prev_angle_y: float = 0.0

    def reset(self) -> None:
        self.has_prev = False
        self.prev_capture_us = 0
        self.prev_angle_x = 0.0
        self.prev_angle_y = 0.0


@dataclass(frozen=True)
class LTMeasurement:
    """Exact mirror of openfc_lt_measurement_t in landing_target_math.h."""

    time_usec: int
    angle_x_rad: float
    angle_y_rad: float
    size_x_rad: float
    size_y_rad: float
    distance_m: float = 0.0
    position_valid: int = 0


def _fill_square(img: bytearray, w: int, h: int, cx: float, cy: float, size: int, fg: int) -> tuple[int, int, int, int]:
    half = size // 2
    x0 = max(0, int(round(cx)) - half)
    x1 = min(w, int(round(cx)) - half + size)
    y0 = max(0, int(round(cy)) - half)
    y1 = min(h, int(round(cy)) - half + size)
    for y in range(y0, y1):
        row_off = y * w
        for x in range(x0, x1):
            img[row_off + x] = fg
    return x0, x1, y0, y1


def make_synthetic_frame(
    w: int,
    h: int,
    kind: str,
    cx: float = 160.0,
    cy: float = 120.0,
    size: int = 34,
    seed: int = 42,
) -> bytearray:
    """Generate deterministic 8-bit grayscale QVGA synthetic test frames."""
    rng = random.Random(seed)

    if kind == "occluded":
        return bytearray([12] * (w * h))
    if kind == "full_dark":
        return bytearray([0] * (w * h))
    if kind == "full_bright":
        return bytearray([255] * (w * h))
    if kind == "full_bright_noisy":
        return bytearray(rng.randint(248, 255) for _ in range(w * h))
    if kind == "low_contrast":
        img = bytearray([132] * (w * h))
        _fill_square(img, w, h, cx, cy, size, 112)
        return img
    if kind == "oversized_dark":
        # > 25% of frame dark -> rejected by max_area_px = total / 4
        img = bytearray([225] * (w * h))
        for y in range(40, 200):
            row_off = y * w
            for x in range(60, 260):
                img[row_off + x] = 18
        return img
    if kind == "thin_streak_h":
        # 120 x 4 horizontal bar: area=480 >= 192, high contrast, aspect=30 > 1.65 -> rejected
        img = bytearray([225] * (w * h))
        for y in range(118, 122):
            row_off = y * w
            for x in range(100, 220):
                img[row_off + x] = 20
        return img
    if kind == "thin_streak_v":
        # 4 x 100 vertical bar: area=400 >= 192, high contrast, aspect=25 > 1.65 -> rejected
        img = bytearray([225] * (w * h))
        for y in range(70, 170):
            row_off = y * w
            for x in range(158, 162):
                img[row_off + x] = 20
        return img
    if kind == "thin_streak_diag":
        # 80-px diagonal 3-px wide streak: aspect=1.0, area~240 >= 192, fill_ratio~0.037 < 0.45 -> rejected
        img = bytearray([225] * (w * h))
        for i in range(80):
            x = 120 + i
            y = 80 + i
            for dx in (-1, 0, 1):
                if 0 <= x + dx < w and 0 <= y < h:
                    img[y * w + (x + dx)] = 15
        return img
    if kind == "salt_pepper_only":
        # Pure salt-and-pepper noise without any marker -> rejected by global fill_ratio < 0.45
        img = bytearray([160] * (w * h))
        for idx in range(w * h):
            r = rng.random()
            if r < 0.04:
                img[idx] = 0
            elif r < 0.08:
                img[idx] = 255
        return img
    if kind == "salt_pepper_fullframe_corrupt":
        # Real marker plus impulsive salt-and-pepper across the entire frame.
        # Under marker_detector.c's global-bbox gate, distant pepper pixels expand the
        # bounding box so fill_ratio < 0.45 -> fail-safe rejection (never emits biased LOS).
        img = bytearray([225] * (w * h))
        _fill_square(img, w, h, cx, cy, size, 20)
        for idx in range(w * h):
            r = rng.random()
            if r < 0.01:
                img[idx] = 0
            elif r < 0.02:
                img[idx] = 255
        return img
    if kind == "salt_pepper_in_marker":
        # Sensor grain on background (strictly above dark threshold) + 10% bright salt
        # dropouts inside the dark marker -> remains accurately detectable (fill_ratio ~ 0.90).
        img = bytearray(rng.randint(200, 240) for _ in range(w * h))
        x0, x1, y0, y1 = _fill_square(img, w, h, cx, cy, size, 22)
        for y in range(y0, y1):
            row_off = y * w
            for x in range(x0, x1):
                if rng.random() < 0.10:
                    img[row_off + x] = 245
        return img

    # Default: nominal high-contrast dark square on light pad
    img = bytearray([225] * (w * h))
    _fill_square(img, w, h, cx, cy, size, 22)
    return img


def detect_dark_marker(
    img: bytearray,
    w: int,
    h: int,
    min_contrast: int = 50,
) -> tuple[bool, float, float, float, float, float]:
    """Exact mirror of openfc_detect_dark_marker() in firmware/esp32-cam/main/marker_detector.c."""
    if w < 8 or h < 8 or len(img) < w * h:
        return False, 0.0, 0.0, 0.0, 0.0, 0.0
    n = w * h
    min_v = min(img)
    max_v = max(img)
    if (max_v - min_v) < min_contrast:
        return False, 0.0, 0.0, 0.0, 0.0, 0.0
    mean_v = sum(img) // n
    thresh = (mean_v + min_v) // 2

    sum_x = sum_y = sum_dark = sum_light = area = light_cnt = 0
    min_x, max_x = w, 0
    min_y, max_y = h, 0
    for y in range(h):
        off = y * w
        for x in range(w):
            p = img[off + x]
            if p <= thresh:
                sum_x += x
                sum_y += y
                sum_dark += p
                area += 1
                if x < min_x:
                    min_x = x
                if x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
            else:
                sum_light += p
                light_cnt += 1

    min_area = max(16, n // 400)
    max_area = n // 4
    if area < min_area or area > max_area or light_cnt == 0:
        return False, 0.0, 0.0, 0.0, 0.0, 0.0
    dark_mean = sum_dark // area
    light_mean = sum_light // light_cnt
    if light_mean <= dark_mean or (light_mean - dark_mean) < min_contrast:
        return False, 0.0, 0.0, 0.0, 0.0, 0.0

    bw = float(max_x - min_x + 1)
    bh = float(max_y - min_y + 1)
    fill = area / (bw * bh)
    aspect = max(bw / bh, bh / bw)
    if fill < 0.45 or aspect > 1.65:
        return False, 0.0, 0.0, 0.0, 0.0, 0.0

    contrast = light_mean - dark_mean
    conf = (contrast / 255.0) * min(1.0, fill)
    return True, sum_x / area, sum_y / area, bw, bh, conf


def project_frd_angles(calib: Calib, u: float, v: float) -> tuple[float, float]:
    """Exact mirror of landing_target_math.c:60-76 matching AC_PrecLand_Companion.cpp:45.

    ArduCopter reconstructs:
      _los_meas_body = Vector3f(-tanf(angle_y), tanf(angle_x), 1.0f)
    """
    cam_right = (u - calib.cx) / calib.fx
    cam_fwd = -(v - calib.cy) / calib.fy
    c = math.cos(calib.yaw_offset_rad)
    s = math.sin(calib.yaw_offset_rad)
    body_fwd = c * cam_fwd - s * cam_right
    body_right = s * cam_fwd + c * cam_right
    return math.atan2(body_right, 1.0), math.atan2(-body_fwd, 1.0)


def arducopter_los_body(angle_x: float, angle_y: float) -> tuple[float, float, float]:
    """Exact ArduCopter AC_PrecLand_Companion.cpp:45-46 unit LOS vector in Body FRD (+X Fwd, +Y Right, +Z Down)."""
    vx = -math.tan(angle_y)
    vy = math.tan(angle_x)
    vz = 1.0
    norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    return vx / norm, vy / norm, vz / norm


def body_ray_to_pixel(calib: Calib, body_fwd: float, body_right: float) -> tuple[float, float]:
    """Inverse pinhole + inverse camera yaw rotation R_z(-yaw) from Body FRD ray (z=+1) to image (u, v)."""
    c = math.cos(calib.yaw_offset_rad)
    s = math.sin(calib.yaw_offset_rad)
    cam_fwd = c * body_fwd + s * body_right
    cam_right = -s * body_fwd + c * body_right
    u = calib.cx + calib.fx * cam_right
    v = calib.cy - calib.fy * cam_fwd
    return u, v


def compute_landing_target(
    calib: Calib,
    flt: FilterState | None,
    detected: bool,
    u_px: float,
    v_px: float,
    box_w_px: float,
    box_h_px: float,
    confidence: float,
    capture_us: int,
    now_us: int,
) -> tuple[LTStatus, LTMeasurement | None]:
    """Exact mirror of openfc_compute_landing_target() in firmware/esp32-cam/main/landing_target_math.c."""
    if (
        not math.isfinite(calib.fx)
        or not math.isfinite(calib.fy)
        or not math.isfinite(calib.cx)
        or not math.isfinite(calib.cy)
        or not math.isfinite(calib.yaw_offset_rad)
        or not math.isfinite(calib.min_confidence)
        or not math.isfinite(calib.max_step_rad)
        or calib.fx <= 0.0
        or calib.fy <= 0.0
        or calib.min_confidence < 0.0
        or calib.min_confidence > 1.0
        or calib.max_step_rad < 0.0
        or calib.width == 0
        or calib.height == 0
    ):
        return LTStatus.ERR_UNCALIBRATED, None
    if not detected:
        return LTStatus.ERR_NOT_DETECTED, None
    if not all(math.isfinite(val) for val in (u_px, v_px, box_w_px, box_h_px, confidence)):
        return LTStatus.ERR_NON_FINITE, None
    if u_px < 0.0 or u_px >= float(calib.width) or v_px < 0.0 or v_px >= float(calib.height):
        return LTStatus.ERR_OUT_OF_BOUNDS, None
    if confidence < calib.min_confidence:
        return LTStatus.ERR_LOW_CONFIDENCE, None

    # Wrap-safe 64-bit unsigned half-space age check (landing_target_math.c:58-62)
    age_us = (now_us - capture_us) & U64_MASK
    if age_us > HALF_U64 or age_us > calib.max_frame_age_us:
        return LTStatus.ERR_STALE_FRAME, None

    cam_right = (u_px - calib.cx) / calib.fx
    cam_fwd = -(v_px - calib.cy) / calib.fy
    c = math.cos(calib.yaw_offset_rad)
    s = math.sin(calib.yaw_offset_rad)
    body_fwd = c * cam_fwd - s * cam_right
    body_right = s * cam_fwd + c * cam_right
    if not all(math.isfinite(val) for val in (cam_right, cam_fwd, body_fwd, body_right)):
        return LTStatus.ERR_NON_FINITE, None

    angle_x = math.atan2(body_right, 1.0)
    angle_y = math.atan2(-body_fwd, 1.0)
    if not math.isfinite(angle_x) or not math.isfinite(angle_y):
        return LTStatus.ERR_NON_FINITE, None

    if flt is not None and flt.has_prev:
        # Wrap-safe monotonic sequence delta; reject out-of-order frames without corrupting filter
        dt = (capture_us - flt.prev_capture_us) & U64_MASK
        if dt > HALF_U64:
            return LTStatus.ERR_STALE_FRAME, None
        if calib.max_step_rad > 0.0 and dt <= calib.outlier_window_us:
            if abs(angle_x - flt.prev_angle_x) > calib.max_step_rad or abs(angle_y - flt.prev_angle_y) > calib.max_step_rad:
                return LTStatus.ERR_OUTLIER_JUMP, None

    size_x = (2.0 * math.atan2(0.5 * box_w_px, calib.fx)) if box_w_px > 0.0 else 0.0
    size_y = (2.0 * math.atan2(0.5 * box_h_px, calib.fy)) if box_h_px > 0.0 else 0.0
    if not math.isfinite(size_x) or not math.isfinite(size_y):
        return LTStatus.ERR_NON_FINITE, None

    if flt is not None:
        flt.has_prev = True
        flt.prev_capture_us = capture_us & U64_MASK
        flt.prev_angle_x = angle_x
        flt.prev_angle_y = angle_y

    return LTStatus.OK, LTMeasurement(
        time_usec=capture_us & U64_MASK,
        angle_x_rad=angle_x,
        angle_y_rad=angle_y,
        size_x_rad=size_x,
        size_y_rad=size_y,
        distance_m=0.0,
        position_valid=0,
    )


def pack_and_verify_landing_target(
    tx: mav.MAVLink,
    rx: mav.MAVLink,
    meas: LTMeasurement,
) -> tuple[bytes, object]:
    """Encode MAVLink2 LANDING_TARGET (#149) and verify byte-by-byte pymavlink round-trip."""
    msg = tx.landing_target_encode(
        meas.time_usec,
        0,
        mav.MAV_FRAME_BODY_FRD,
        meas.angle_x_rad,
        meas.angle_y_rad,
        meas.distance_m,
        meas.size_x_rad,
        meas.size_y_rad,
        0.0,
        0.0,
        0.0,
        [1.0, 0.0, 0.0, 0.0],
        mav.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        meas.position_valid,
    )
    raw = msg.pack(tx)
    parsed = None
    for b in raw:
        m = rx.parse_char(bytes([b]))
        if m is not None:
            parsed = m
    if parsed is None or parsed.get_msgId() != mav.MAVLINK_MSG_ID_LANDING_TARGET:
        raise AssertionError("pymavlink round-trip parse failed for LANDING_TARGET")
    if parsed.frame != mav.MAV_FRAME_BODY_FRD or parsed.position_valid != 0:
        raise AssertionError(f"invalid MAVLink2 fields: frame={parsed.frame} pos_valid={parsed.position_valid}")
    return raw, parsed


def verify_golden_mavlink2_vectors() -> None:
    """Verify bit-exact parity between pymavlink and C mavlink2_tx.c golden vectors."""
    tx_hb = mav.MAVLink(None, srcSystem=1, srcComponent=191)
    hb_raw = tx_hb.heartbeat_encode(
        mav.MAV_TYPE_ONBOARD_CONTROLLER,
        mav.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mav.MAV_STATE_ACTIVE,
    ).pack(tx_hb)
    assert hb_raw == GOLDEN_MAVLINK2_HEARTBEAT, f"HEARTBEAT golden mismatch: {hb_raw.hex()}"

    tx_lt = mav.MAVLink(None, srcSystem=1, srcComponent=191)
    lt_raw = tx_lt.landing_target_encode(
        12_345_678,
        0,
        mav.MAV_FRAME_BODY_FRD,
        0.05,
        -0.03,
        2.5,
        0.1,
        0.1,
        0.0,
        0.0,
        0.0,
        [1.0, 0.0, 0.0, 0.0],
        mav.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        0,
    ).pack(tx_lt)
    assert lt_raw == GOLDEN_MAVLINK2_LANDING_TARGET, f"LANDING_TARGET golden mismatch: {lt_raw.hex()}"

    # Verify seq=255 -> 0 rollover matches C mavlink2_tx.c golden vector byte-for-byte
    # Note: pymavlink's msg.pack(tx) reads tx.seq without mutating it (tx.send() wraps (seq+1)%256).
    tx_lt.seq = 255
    lt255_raw = tx_lt.landing_target_encode(
        0,
        0,
        mav.MAV_FRAME_BODY_FRD,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        [1.0, 0.0, 0.0, 0.0],
        mav.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        0,
    ).pack(tx_lt)
    assert lt255_raw == GOLDEN_MAVLINK2_LANDING_TARGET_SEQ255, f"LANDING_TARGET seq255 mismatch: {lt255_raw.hex()}"
    assert lt255_raw[4] == 255
    tx_lt.seq = (tx_lt.seq + 1) & 0xFF
    assert tx_lt.seq == 0
    lt0_raw = tx_lt.landing_target_encode(
        0,
        0,
        mav.MAV_FRAME_BODY_FRD,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        [1.0, 0.0, 0.0, 0.0],
        mav.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        0,
    ).pack(tx_lt)
    assert lt0_raw[4] == 0 and len(lt0_raw) == 71


def verify_yaw_offset_rotations() -> dict:
    """Verify camera yaw offsets (0, 90, 180, 270 deg) directly against AC_PrecLand_Companion.cpp:45.

    Uses distinct fx != fy (260.0 vs 235.0) to also catch any fx/fy axis-swap bugs across 90/270 deg,
    plus both on-grid and sub-pixel target rays.
    """
    tx = mav.MAVLink(None, srcSystem=1, srcComponent=191)
    rx = mav.MAVLink(None)
    # Body FRD test rays (body_fwd, body_right) at z=+1m (kept well within QVGA FOV)
    body_rays = [
        ("forward", 0.26, 0.00),
        ("right", 0.00, 0.26),
        ("back", -0.26, 0.00),
        ("left", 0.00, -0.26),
        ("fwd_right", 0.22, 0.18),
        ("back_left", -0.20, -0.22),
        ("subpx_a", 0.173, -0.237),
        ("subpx_b", -0.191, 0.147),
    ]
    checked = 0
    max_analytic_err = 0.0
    max_img_err = 0.0

    for yaw_deg in (0, 90, 180, 270):
        calib = Calib(fx=260.0, fy=235.0, yaw_offset_rad=math.radians(yaw_deg))
        for name, b_fwd, b_right in body_rays:
            # Ground-truth unit LOS vector in ArduCopter Body FRD
            gt_norm = math.sqrt(b_fwd * b_fwd + b_right * b_right + 1.0)
            gt_los = (b_fwd / gt_norm, b_right / gt_norm, 1.0 / gt_norm)

            # 1) Analytic exactness check
            u_gt, v_gt = body_ray_to_pixel(calib, b_fwd, b_right)
            ax_a, ay_a = project_frd_angles(calib, u_gt, v_gt)
            los_a = arducopter_los_body(ax_a, ay_a)
            err_a = math.sqrt(sum((los_a[i] - gt_los[i]) ** 2 for i in range(3)))
            max_analytic_err = max(max_analytic_err, err_a)
            assert err_a < 1e-12, f"yaw={yaw_deg} {name}: analytic LOS mismatch {err_a}"

            # 2) Full synthetic image -> detector -> compute_landing_target -> MAVLink2 -> AC_PrecLand LOS
            img = make_synthetic_frame(calib.width, calib.height, "nominal", u_gt, v_gt, size=35)
            ok, cx, cy, bw, bh, conf = detect_dark_marker(img, calib.width, calib.height)
            assert ok, f"yaw={yaw_deg} {name}: marker not detected at ({u_gt:.1f}, {v_gt:.1f})"
            st, meas = compute_landing_target(calib, None, ok, cx, cy, bw, bh, conf, 1_000_000, 1_015_000)
            assert st == LTStatus.OK and meas is not None, f"yaw={yaw_deg} {name}: status={st}"
            _, parsed = pack_and_verify_landing_target(tx, rx, meas)
            los_m = arducopter_los_body(parsed.angle_x, parsed.angle_y)
            err_m = math.sqrt(sum((los_m[i] - gt_los[i]) ** 2 for i in range(3)))
            max_img_err = max(max_img_err, err_m)
            assert err_m < 0.008, f"yaw={yaw_deg} {name}: image LOS error {err_m:.5f} >= 0.008"

            # Explicit sign checks for non-zero axes
            if abs(b_fwd) > 0.05:
                assert math.copysign(1.0, los_m[0]) == math.copysign(1.0, b_fwd), (
                    f"yaw={yaw_deg} {name}: X sign mismatch got {los_m[0]} vs {b_fwd}"
                )
            if abs(b_right) > 0.05:
                assert math.copysign(1.0, los_m[1]) == math.copysign(1.0, b_right), (
                    f"yaw={yaw_deg} {name}: Y sign mismatch got {los_m[1]} vs {b_right}"
                )
            checked += 1

    # Negative control: a 180-deg wrong yaw offset MUST invert horizontal LOS signs
    c0 = Calib(fx=260.0, fy=235.0, yaw_offset_rad=0.0)
    c180 = Calib(fx=260.0, fy=235.0, yaw_offset_rad=math.pi)
    u_ref, v_ref = body_ray_to_pixel(c0, 0.22, 0.18)
    los_wrong = arducopter_los_body(*project_frd_angles(c180, u_ref, v_ref))
    assert los_wrong[0] < 0.0 and los_wrong[1] < 0.0, "negative control failed: 180-deg wrong yaw must invert LOS"

    return {
        "yaw_angles_deg": [0, 90, 180, 270],
        "cases_verified": checked,
        "max_analytic_los_err": round(max_analytic_err, 12),
        "max_pipeline_los_err": round(max_img_err, 6),
    }


def verify_stress_and_gate_cases() -> dict:
    """Verify stress frames, rejection gates, and post-occlusion re-acquisition."""
    calib = Calib()
    tx = mav.MAVLink(None, srcSystem=1, srcComponent=191)
    rx = mav.MAVLink(None)
    checks_passed = 0

    # 1) Rejection gate parity checks (uncalibrated, config NaN/range, OOB, non-finite, stale, wraparound, out-of-order)
    for bad_calib in (
        Calib(fx=0.0),
        Calib(fy=-10.0),
        Calib(min_confidence=float("nan")),
        Calib(min_confidence=-0.1),
        Calib(min_confidence=1.1),
        Calib(max_step_rad=float("nan")),
        Calib(max_step_rad=-0.1),
    ):
        st, _ = compute_landing_target(bad_calib, None, True, 160.0, 120.0, 30.0, 30.0, 0.8, 1000, 2000)
        assert st == LTStatus.ERR_UNCALIBRATED, f"expected ERR_UNCALIBRATED for {bad_calib}, got {st}"
    # Subnormal fx causing Inf ray must be rejected as ERR_NON_FINITE before atan2
    st, _ = compute_landing_target(Calib(fx=1e-320), None, True, 210.0, 70.0, 30.0, 30.0, 0.8, 1000, 2000)
    assert st == LTStatus.ERR_NON_FINITE
    for bad_u, bad_v in ((-0.1, 120.0), (320.0, 120.0), (160.0, -0.1), (160.0, 240.0)):
        st, _ = compute_landing_target(calib, None, True, bad_u, bad_v, 30.0, 30.0, 0.8, 1000, 2000)
        assert st == LTStatus.ERR_OUT_OF_BOUNDS
    for bad_val in (float("nan"), float("inf"), float("-inf")):
        st, _ = compute_landing_target(calib, None, True, bad_val, 120.0, 30.0, 30.0, 0.8, 1000, 2000)
        assert st == LTStatus.ERR_NON_FINITE
    st, _ = compute_landing_target(calib, None, True, 160.0, 120.0, 30.0, 30.0, 0.8, 5000, 4000)
    assert st == LTStatus.ERR_STALE_FRAME
    st, _ = compute_landing_target(calib, None, True, 160.0, 120.0, 30.0, 30.0, 0.05, 1000, 2000)
    assert st == LTStatus.ERR_LOW_CONFIDENCE

    # Out-of-order older capture_us & 64-bit unsigned wraparound safety (landing_target_math.c:58-108)
    wrap_flt = FilterState()
    st, _ = compute_landing_target(calib, wrap_flt, True, 210.0, 70.0, 30.0, 30.0, 0.85, 1_000_000, 1_010_000)
    assert st == LTStatus.OK and wrap_flt.prev_capture_us == 1_000_000
    st, _ = compute_landing_target(calib, wrap_flt, True, 210.0, 70.0, 30.0, 30.0, 0.85, 950_000, 960_000)
    assert st == LTStatus.ERR_STALE_FRAME and wrap_flt.prev_capture_us == 1_000_000

    wrap_flt.reset()
    near_max = U64_MASK - 10_000
    st, _ = compute_landing_target(calib, wrap_flt, True, 210.0, 70.0, 30.0, 30.0, 0.85, near_max, 10_000)
    assert st == LTStatus.OK and wrap_flt.prev_capture_us == near_max
    st, _ = compute_landing_target(calib, wrap_flt, True, 210.0, 70.0, 30.0, 30.0, 0.85, near_max, 200_000)
    assert st == LTStatus.ERR_STALE_FRAME
    st, _ = compute_landing_target(calib, wrap_flt, True, 20.0, 220.0, 30.0, 30.0, 0.85, 40_000, 50_000)
    assert st == LTStatus.ERR_OUTLIER_JUMP
    st, _ = compute_landing_target(calib, wrap_flt, True, 20.0, 220.0, 30.0, 30.0, 0.85, 400_000, 410_000)
    assert st == LTStatus.OK and wrap_flt.prev_capture_us == 400_000
    checks_passed += 1

    # 2) Near-border off-center markers (all 4 corners + clipped corner)
    border_targets = [
        ("top_left", 22.0, 22.0, 35, +1.0, -1.0),      # +X Fwd, -Y Left
        ("top_right", 298.0, 22.0, 35, +1.0, +1.0),    # +X Fwd, +Y Right
        ("bottom_left", 22.0, 218.0, 35, -1.0, -1.0),  # -X Back, -Y Left
        ("bottom_right", 298.0, 218.0, 35, -1.0, +1.0),# -X Back, +Y Right
        ("clipped_tl", 10.0, 10.0, 35, +1.0, -1.0),    # Clipped at x=0, y=0 -> still valid square
    ]
    for name, u, v, sz, sign_x, sign_y in border_targets:
        img = make_synthetic_frame(calib.width, calib.height, "nominal", u, v, size=sz)
        ok, cx, cy, bw, bh, conf = detect_dark_marker(img, calib.width, calib.height)
        assert ok, f"near-border {name} failed detection"
        st, meas = compute_landing_target(calib, None, ok, cx, cy, bw, bh, conf, 1_000_000, 1_020_000)
        assert st == LTStatus.OK and meas is not None, f"near-border {name} failed compute: {st}"
        _, parsed = pack_and_verify_landing_target(tx, rx, meas)
        los_x, los_y, los_z = arducopter_los_body(parsed.angle_x, parsed.angle_y)
        assert math.copysign(1.0, los_x) == sign_x and math.copysign(1.0, los_y) == sign_y and los_z > 0.0
    checks_passed += 1

    # 3) Negative false-marker & extreme illumination stress cases -> zero false positives
    negative_kinds = [
        "salt_pepper_only",
        "salt_pepper_fullframe_corrupt",
        "thin_streak_h",
        "thin_streak_v",
        "thin_streak_diag",
        "full_dark",
        "occluded",
        "full_bright",
        "full_bright_noisy",
        "oversized_dark",
        "low_contrast",
    ]
    for idx, kind in enumerate(negative_kinds):
        img = make_synthetic_frame(calib.width, calib.height, kind, 160.0, 120.0, size=35, seed=100 + idx)
        ok, *_ = detect_dark_marker(img, calib.width, calib.height)
        assert not ok, f"negative stress case {kind} falsely triggered detector"
    checks_passed += 1

    # 4) Positive salt-in-marker + background grain stress case -> accurately detected
    img = make_synthetic_frame(calib.width, calib.height, "salt_pepper_in_marker", 195.0, 92.0, size=35, seed=77)
    ok, cx, cy, bw, bh, conf = detect_dark_marker(img, calib.width, calib.height)
    assert ok, "salt_pepper_in_marker should remain detectable"
    st, meas = compute_landing_target(calib, None, ok, cx, cy, bw, bh, conf, 2_000_000, 2_020_000)
    assert st == LTStatus.OK and meas is not None
    gt_ax, gt_ay = project_frd_angles(calib, 195.0, 92.0)
    assert math.hypot(meas.angle_x_rad - gt_ax, meas.angle_y_rad - gt_ay) < 0.01
    checks_passed += 1

    # 5) Target re-acquisition after sustained occlusion (> outlier_window_us = 300 ms)
    flt = FilterState()
    t_us = 5_000_000
    # Anchor frame top-right
    img = make_synthetic_frame(calib.width, calib.height, "nominal", 240.0, 60.0, size=35)
    ok, cx, cy, bw, bh, conf = detect_dark_marker(img, calib.width, calib.height)
    st, meas = compute_landing_target(calib, flt, ok, cx, cy, bw, bh, conf, t_us, t_us + 15_000)
    assert st == LTStatus.OK and meas is not None
    # Sustained occlusion: 8 frames over 400 ms
    for _ in range(8):
        t_us += 50_000
        occ = make_synthetic_frame(calib.width, calib.height, "occluded")
        ok, cx, cy, bw, bh, conf = detect_dark_marker(occ, calib.width, calib.height)
        st, _ = compute_landing_target(calib, flt, ok, cx, cy, bw, bh, conf, t_us, t_us + 15_000)
        assert st == LTStatus.ERR_NOT_DETECTED
    # Re-appear at distant bottom-left (> 0.6 rad jump, dt = 450 ms > 300 ms) -> MUST re-acquire immediately
    t_us += 50_000
    img = make_synthetic_frame(calib.width, calib.height, "nominal", 65.0, 195.0, size=35)
    ok, cx, cy, bw, bh, conf = detect_dark_marker(img, calib.width, calib.height)
    st, meas = compute_landing_target(calib, flt, ok, cx, cy, bw, bh, conf, t_us, t_us + 15_000)
    assert st == LTStatus.OK and meas is not None, f"failed post-occlusion re-acquisition: {st}"

    # Single-frame outlier glitch must NOT poison filter state
    t_us += 50_000
    glitch = make_synthetic_frame(calib.width, calib.height, "nominal", 270.0, 40.0, size=35)
    ok, cx, cy, bw, bh, conf = detect_dark_marker(glitch, calib.width, calib.height)
    st, _ = compute_landing_target(calib, flt, ok, cx, cy, bw, bh, conf, t_us, t_us + 15_000)
    assert st == LTStatus.ERR_OUTLIER_JUMP
    t_us += 50_000
    back = make_synthetic_frame(calib.width, calib.height, "nominal", 72.0, 190.0, size=35)
    ok, cx, cy, bw, bh, conf = detect_dark_marker(back, calib.width, calib.height)
    st, meas = compute_landing_target(calib, flt, ok, cx, cy, bw, bh, conf, t_us, t_us + 15_000)
    assert st == LTStatus.OK and meas is not None, "valid track after single-frame glitch must recover immediately"

    # Brief occlusion (100 ms < 300 ms) followed by sustained new position -> must NOT permanently lock out
    t_us += 100_000
    reacquired_after_window = False
    for step in range(8):
        img = make_synthetic_frame(calib.width, calib.height, "nominal", 250.0, 55.0, size=35)
        ok, cx, cy, bw, bh, conf = detect_dark_marker(img, calib.width, calib.height)
        st, meas = compute_landing_target(calib, flt, ok, cx, cy, bw, bh, conf, t_us, t_us + 15_000)
        if step <= 4:  # dt = 100, 150, 200, 250, 300 ms <= 300 ms
            assert st == LTStatus.ERR_OUTLIER_JUMP, f"step {step} expected ERR_OUTLIER_JUMP, got {st}"
        else:  # dt = 350 ms > 300 ms -> re-acquires and tracks cleanly
            assert st == LTStatus.OK and meas is not None, f"step {step} expected OK re-acquisition, got {st}"
            reacquired_after_window = True
        t_us += 50_000
    assert reacquired_after_window
    checks_passed += 1

    return {
        "stress_groups_passed": checks_passed,
        "negative_stress_rejected": len(negative_kinds),
        "near_border_verified": len(border_targets),
        "post_occlusion_reacquisition": "PASS",
    }


def verify_closed_loop_descent() -> dict:
    """Closed-loop 2D descent convergence test under all 4 camera yaw offsets (0, 90, 180, 270 deg).

    Vehicle starts offset at (x_d=+0.85 m, y_d=-0.60 m, z=4.5 m) relative to pad at (0, 0, 0).
    At each frame:
      1. Render synthetic 320x240 frame from pinhole + camera yaw offset
      2. Run marker detector + landing_target_math + MAVLink2 pack & wire parse
      3. Apply ArduCopter AC_PrecLand_Companion.cpp:45 LOS body vector to reduce lateral error
      4. Assert strict monotonic lateral error reduction at every single descent step
    """
    tx = mav.MAVLink(None, srcSystem=1, srcComponent=191)
    rx = mav.MAVLink(None)
    per_yaw: dict[str, dict] = {}
    steps = 14
    k_p = 0.28
    dz = 0.25
    pad_size_m = 0.45

    for yaw_deg in (0, 90, 180, 270):
        calib = Calib(yaw_offset_rad=math.radians(yaw_deg))
        flt = FilterState()
        x_d = 0.85   # +X body (drone is 0.85m forward of pad -> pad is -0.85m behind drone)
        y_d = -0.60  # -Y body (drone is 0.60m left of pad -> pad is +0.60m right of drone)
        z = 4.50
        t_us = 10_000_000
        init_err = math.hypot(x_d, y_d)
        prev_err = init_err

        for step in range(steps):
            # True pad offset in Body FRD relative to vehicle: (-x_d, -y_d, +z)
            body_fwd = -x_d / z
            body_right = -y_d / z
            u_gt, v_gt = body_ray_to_pixel(calib, body_fwd, body_right)
            # Odd marker size scaled by altitude so rendered square has zero -0.5px bias
            px_size = max(21, min(65, int(round(pad_size_m * calib.fx / z)) | 1))
            img = make_synthetic_frame(calib.width, calib.height, "nominal", u_gt, v_gt, size=px_size)

            ok, cx, cy, bw, bh, conf = detect_dark_marker(img, calib.width, calib.height)
            assert ok, f"descent yaw={yaw_deg} step={step}: marker lost at ({u_gt:.1f}, {v_gt:.1f})"
            st, meas = compute_landing_target(calib, flt, ok, cx, cy, bw, bh, conf, t_us, t_us + 20_000)
            assert st == LTStatus.OK and meas is not None, f"descent yaw={yaw_deg} step={step}: status={st}"

            _, parsed = pack_and_verify_landing_target(tx, rx, meas)
            los_x, los_y, los_z = arducopter_los_body(parsed.angle_x, parsed.angle_y)

            # Estimated target offset from drone in Body FRD using ArduCopter's LOS vector
            est_dx = z * (los_x / los_z)
            est_dy = z * (los_y / los_z)

            # Move drone toward estimated target position and descend
            x_d += k_p * est_dx
            y_d += k_p * est_dy
            z = max(1.0, z - dz)
            t_us += 50_000

            err = math.hypot(x_d, y_d)
            assert err < prev_err, (
                f"descent yaw={yaw_deg} step={step}: lateral error did not decrease "
                f"monotonically ({err:.4f}m >= {prev_err:.4f}m)"
            )
            prev_err = err

        assert prev_err < 0.025, f"descent yaw={yaw_deg}: failed to converge (<0.025m), got {prev_err:.4f}m"
        per_yaw[f"{yaw_deg}deg"] = {
            "steps": steps,
            "initial_err_m": round(init_err, 4),
            "final_err_m": round(prev_err, 4),
            "final_alt_m": round(z, 2),
            "monotonic": True,
        }

    return per_yaw


def build_dataset(calib: Calib) -> list[dict]:
    """Deterministic synthetic replay dataset covering nominal, stress, bad, outlier, and post-occlusion frames."""
    frames: list[dict] = []
    t_us = 1_000_000

    # 30 nominal frames smoothly converging from offset (+45px, -35px) toward center
    for i in range(30):
        frac = 1.0 - (i / 29.0)
        ux = calib.cx + 45.0 * frac * math.cos(i * 0.15)
        vy = calib.cy - 35.0 * frac * math.sin(i * 0.15 + 0.5)
        frames.append({
            "idx": len(frames),
            "label": "nominal",
            "expect_emit": True,
            "kind": "nominal",
            "gt_u": ux,
            "gt_v": vy,
            "capture_us": t_us,
            "process_us": t_us + 22_000,
        })
        t_us += 50_000

    # 2 nominal frames with in-marker salt speckles + background grain (must remain detected)
    for i in range(2):
        frames.append({
            "idx": len(frames),
            "label": "salt_pepper_in_marker",
            "expect_emit": True,
            "kind": "salt_pepper_in_marker",
            "gt_u": calib.cx + 12.0 * (i + 1),
            "gt_v": calib.cy - 10.0 * (i + 1),
            "capture_us": t_us,
            "process_us": t_us + 22_000,
        })
        t_us += 50_000

    # 6 occluded blackout frames
    for _ in range(6):
        frames.append({
            "idx": len(frames),
            "label": "occluded",
            "expect_emit": False,
            "kind": "occluded",
            "gt_u": calib.cx,
            "gt_v": calib.cy,
            "capture_us": t_us,
            "process_us": t_us + 20_000,
        })
        t_us += 50_000

    # 4 full-frame darkness / brightness frames
    for kind in ("full_dark", "full_dark", "full_bright", "full_bright_noisy"):
        frames.append({
            "idx": len(frames),
            "label": kind,
            "expect_emit": False,
            "kind": kind,
            "gt_u": calib.cx,
            "gt_v": calib.cy,
            "capture_us": t_us,
            "process_us": t_us + 20_000,
        })
        t_us += 50_000

    # 5 low-contrast blurred frames
    for _ in range(5):
        frames.append({
            "idx": len(frames),
            "label": "low_contrast",
            "expect_emit": False,
            "kind": "low_contrast",
            "gt_u": calib.cx,
            "gt_v": calib.cy,
            "capture_us": t_us,
            "process_us": t_us + 20_000,
        })
        t_us += 50_000

    # 4 thin-streak / oversized false-marker frames
    for kind in ("thin_streak_h", "thin_streak_v", "thin_streak_diag", "oversized_dark"):
        frames.append({
            "idx": len(frames),
            "label": kind,
            "expect_emit": False,
            "kind": kind,
            "gt_u": calib.cx,
            "gt_v": calib.cy,
            "capture_us": t_us,
            "process_us": t_us + 20_000,
        })
        t_us += 50_000

    # 4 salt-and-pepper rejection frames (2 pure noise, 2 full-frame impulsive corruption)
    for kind in (
        "salt_pepper_only",
        "salt_pepper_only",
        "salt_pepper_fullframe_corrupt",
        "salt_pepper_fullframe_corrupt",
    ):
        frames.append({
            "idx": len(frames),
            "label": kind,
            "expect_emit": False,
            "kind": kind,
            "gt_u": calib.cx,
            "gt_v": calib.cy,
            "capture_us": t_us,
            "process_us": t_us + 20_000,
        })
        t_us += 50_000

    # 5 stale-timestamp frames (age = 240 ms > 150 ms limit)
    for _ in range(5):
        frames.append({
            "idx": len(frames),
            "label": "stale",
            "expect_emit": False,
            "kind": "nominal",
            "gt_u": calib.cx + 5.0,
            "gt_v": calib.cy - 5.0,
            "capture_us": t_us,
            "process_us": t_us + 240_000,
        })
        t_us += 50_000

    # 1 clean anchor followed by 4 sudden-outlier jump frames (> 0.35 rad jump within 50 ms)
    t_us += 400_000  # fresh window after bad frames
    frames.append({
        "idx": len(frames),
        "label": "nominal_anchor",
        "expect_emit": True,
        "kind": "nominal",
        "gt_u": calib.cx,
        "gt_v": calib.cy,
        "capture_us": t_us,
        "process_us": t_us + 18_000,
    })
    t_us += 50_000
    for _ in range(4):
        frames.append({
            "idx": len(frames),
            "label": "outlier",
            "expect_emit": False,
            "kind": "nominal",
            "gt_u": 25.0,   # > 0.49 rad jump from center
            "gt_v": 215.0,
            "capture_us": t_us,
            "process_us": t_us + 18_000,
        })
        t_us += 50_000

    # Sustained occlusion (8 frames = 400 ms > 300 ms outlier window) -> distant near-border re-acquisition
    for _ in range(8):
        frames.append({
            "idx": len(frames),
            "label": "sustained_occlusion",
            "expect_emit": False,
            "kind": "occluded",
            "gt_u": calib.cx,
            "gt_v": calib.cy,
            "capture_us": t_us,
            "process_us": t_us + 18_000,
        })
        t_us += 50_000

    # 4 re-acquired frames starting near top-left border (28, 28) and tracking inward
    for i in range(4):
        frames.append({
            "idx": len(frames),
            "label": "reacquired_near_border",
            "expect_emit": True,
            "kind": "nominal",
            "gt_u": 28.0 + i * 8.0,
            "gt_v": 28.0 + i * 6.0,
            "capture_us": t_us,
            "process_us": t_us + 18_000,
        })
        t_us += 50_000

    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description="OPEN-FC synthetic vision replay & MAVLink2 simulation benchmark")
    parser.add_argument("--udp", default=None, help="Optional host:port to stream live MAVLink2 UDP to SITL")
    parser.add_argument("--report", default=None, help="Optional path to write JSON benchmark summary")
    args = parser.parse_args()

    # 1. Golden MAVLink2 byte parity (C mavlink2_tx.c <-> pymavlink, incl. seq 255->0 rollover)
    verify_golden_mavlink2_vectors()
    print(
        "  [PASS] golden_mavlink2_vectors : HEARTBEAT (21B), LANDING_TARGET (71B), & seq255->0 rollover bit-exact with C mavlink2_tx.c"
    )

    # 2. Camera yaw-offset rotation verification (0, 90, 180, 270 deg) vs AC_PrecLand_Companion.cpp:45
    yaw_summary = verify_yaw_offset_rotations()
    print(
        f"  [PASS] yaw_offset_rotation     : {yaw_summary['cases_verified']}/{yaw_summary['cases_verified']} cases across "
        f"0/90/180/270 deg (max_los_err={yaw_summary['max_pipeline_los_err']} rad)"
    )

    # 3. Synthetic stress cases, rejection gates & post-occlusion re-acquisition
    stress_summary = verify_stress_and_gate_cases()
    print(
        f"  [PASS] synthetic_stress_suite  : {stress_summary['stress_groups_passed']}/5 groups "
        f"({stress_summary['negative_stress_rejected']} false-marker/illum rejects, "
        f"{stress_summary['near_border_verified']} near-border corners, post-occlusion re-acq OK)"
    )

    # 4. Closed-loop 2D descent convergence across all 4 yaw offsets
    descent_summary = verify_closed_loop_descent()
    descent_brief = ", ".join(
        f"{k}:{v['initial_err_m']}m->{v['final_err_m']}m" for k, v in descent_summary.items()
    )
    print(f"  [PASS] closed_loop_2d_descent  : 4/4 yaw offsets converged monotonically ({descent_brief})")

    # 5. Full sequential synthetic replay dataset
    calib = Calib()
    dataset = build_dataset(calib)

    tx = mav.MAVLink(None, srcSystem=1, srcComponent=191)
    rx = mav.MAVLink(None)

    sock = None
    udp_dst = None
    if args.udp:
        host, port_s = args.udp.rsplit(":", 1)
        udp_dst = (host, int(port_s))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    flt = FilterState()
    emitted_valid = 0
    emitted_false_positive = 0
    missed_nominal = 0
    rejected_bad = 0
    ang_errors: list[float] = []
    host_sim_proc_times_ms: list[float] = []

    for item in dataset:
        img = make_synthetic_frame(
            calib.width,
            calib.height,
            item["kind"],
            item["gt_u"],
            item["gt_v"],
            size=34,
            seed=1000 + item["idx"],
        )
        t0 = time.perf_counter()
        ok, cx, cy, bw, bh, conf = detect_dark_marker(img, calib.width, calib.height)
        st, meas = compute_landing_target(
            calib, flt, ok, cx, cy, bw, bh, conf, item["capture_us"], item["process_us"]
        )
        host_sim_proc_times_ms.append((time.perf_counter() - t0) * 1000.0)

        if st == LTStatus.OK and meas is not None:
            raw, parsed = pack_and_verify_landing_target(tx, rx, meas)
            if sock is not None and udp_dst is not None:
                sock.sendto(raw, udp_dst)

            if item["expect_emit"]:
                emitted_valid += 1
                gt_ax, gt_ay = project_frd_angles(calib, item["gt_u"], item["gt_v"])
                ang_errors.append(math.hypot(parsed.angle_x - gt_ax, parsed.angle_y - gt_ay))
            else:
                emitted_false_positive += 1
        else:
            if item["expect_emit"]:
                missed_nominal += 1
            else:
                rejected_bad += 1

    if sock is not None:
        sock.close()

    total_nominal = sum(1 for d in dataset if d["expect_emit"])
    total_bad = len(dataset) - total_nominal
    mae_rad = (sum(ang_errors) / len(ang_errors)) if ang_errors else float("nan")
    max_err_rad = max(ang_errors) if ang_errors else float("nan")
    host_sim_avg_ms = sum(host_sim_proc_times_ms) / len(host_sim_proc_times_ms)

    summary = {
        "dataset": f"synthetic_qvga_320x240_{len(dataset)}frames",
        "timing_scope": "host_python_simulation_only",
        "note": "Host Python simulation benchmark ONLY (NOT ESP32-CAM hardware FPS — AGENTS.md §3.2)",
        "nominal_detected": f"{emitted_valid}/{total_nominal}",
        "bad_frames_rejected": f"{rejected_bad}/{total_bad}",
        "false_positives": emitted_false_positive,
        "missed_nominal": missed_nominal,
        "angular_mae_rad": round(mae_rad, 6),
        "angular_max_err_rad": round(max_err_rad, 6),
        "host_python_sim_avg_frame_ms": round(host_sim_avg_ms, 3),
        "yaw_offset_verification": yaw_summary,
        "stress_verification": stress_summary,
        "closed_loop_descent_verification": descent_summary,
    }

    print("Vision Replay Benchmark (HOST PYTHON SIMULATION ONLY — NOT ESP32-CAM hardware FPS):")
    for k in (
        "dataset",
        "timing_scope",
        "note",
        "nominal_detected",
        "bad_frames_rejected",
        "false_positives",
        "missed_nominal",
        "angular_mae_rad",
        "angular_max_err_rad",
        "host_python_sim_avg_frame_ms",
    ):
        print(f"  {k}: {summary[k]}")

    if args.report:
        Path(args.report).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    if emitted_false_positive != 0 or missed_nominal != 0 or max_err_rad > 0.01:
        print("FAIL: vision replay assertions failed.", file=sys.stderr)
        return 1

    print(
        f"PASS: Vision simulation suite green ({emitted_valid}/{total_nominal} nominal, "
        f"{rejected_bad}/{total_bad} bad/stress rejected, 4/4 yaw offsets, 4/4 closed-loop descents, 0 invalid packets)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
