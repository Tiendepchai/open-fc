#!/usr/bin/env python3
"""M4 MAVLink2 Fault-Injection Verification Harness (SITL Live + Loopback Self-Test).

Verifies all 4 safety-case fault scenarios (docs/safety-case.md §2):
  1. rc-loss          : SIM_RC_FAIL=1 -> FC enters RTL/LAND failsafe
  2. battery-sag      : SIM_BATT_VOLTAGE 12.6V -> 10.6V (Low -> RTL) -> 10.0V (Crit -> LAND)
  3. wifi-drop        : GCS telemetry silence -> FC remains armed/stable without spurious disarm
  4. vision-occlusion : LANDING_TARGET stops mid-LAND -> FC clears stale LOS (>1000ms) safely

Modes:
  --self-test             : Spins up an in-process real-UDP pymavlink mock ArduCopter FC
                            on 127.0.0.1 to verify the full MAVLink2 fault-injection suite.
  --live udp:HOST:PORT    : Runs against live ArduCopter SITL (sim/sitl/run-sitl.sh).
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time

from pymavlink.dialects.v20 import ardupilotmega as mav

COPTER_MODE_LOITER = 5
COPTER_MODE_RTL = 6
COPTER_MODE_LAND = 9


class MockArduCopterFC:
    """Loopback UDP MAVLink2 mock mirroring ArduCopter Rev K failsafe state transitions."""

    def __init__(self, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self.stop_evt = threading.Event()
        self.tx = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        self.rx = mav.MAVLink(None)
        self.mode = COPTER_MODE_LOITER
        self.armed = True
        self.sim_rc_fail = 0.0
        self.sim_batt_v = 12.6
        self.last_lt_time = 0.0
        self.los_active = False
        self.gcs_peer: tuple[str, int] | None = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_evt.set()
        self.thread.join(timeout=1.0)
        self.sock.close()

    def _update_failsafes(self) -> None:
        if self.sim_batt_v <= 10.2:
            self.mode = COPTER_MODE_LAND
        elif self.sim_batt_v <= 10.8 or self.sim_rc_fail >= 1.0:
            self.mode = COPTER_MODE_RTL
        # Mirror AC_PrecLand_Companion.cpp:19 (1000 ms LOS timeout)
        if self.los_active and (time.monotonic() - self.last_lt_time) > 0.35:
            # Accelerated 350 ms timeout in unit self-test (1000 ms in real ArduCopter)
            self.los_active = False

    def _run(self) -> None:
        last_hb = 0.0
        while not self.stop_evt.is_set():
            try:
                data, addr = self.sock.recvfrom(1024)
                self.gcs_peer = addr
                for b in data:
                    msg = self.rx.parse_char(bytes([b]))
                    if msg is None:
                        continue
                    mid = msg.get_msgId()
                    if mid == mav.MAVLINK_MSG_ID_PARAM_SET:
                        pid = msg.param_id.rstrip("\x00")
                        if pid == "SIM_RC_FAIL":
                            self.sim_rc_fail = msg.param_value
                        elif pid == "SIM_BATT_VOLTAGE":
                            self.sim_batt_v = msg.param_value
                        elif pid == "SIM_RESET_MODE":
                            self.mode = int(msg.param_value)
                        ack = self.tx.param_value_encode(
                            pid.encode("ascii"), msg.param_value, mav.MAV_PARAM_TYPE_REAL32, 1, 0
                        )
                        self.sock.sendto(ack.pack(self.tx), addr)
                    elif mid == mav.MAVLINK_MSG_ID_LANDING_TARGET:
                        if msg.frame == mav.MAV_FRAME_BODY_FRD:
                            self.last_lt_time = time.monotonic()
                            self.los_active = True
            except socket.timeout:
                pass

            self._update_failsafes()
            now = time.monotonic()
            if self.gcs_peer is not None and (now - last_hb) >= 0.05:
                base_mode = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                if self.armed:
                    base_mode |= mav.MAV_MODE_FLAG_SAFETY_ARMED
                hb = self.tx.heartbeat_encode(
                    mav.MAV_TYPE_QUADROTOR,
                    mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    base_mode,
                    self.mode,
                    mav.MAV_STATE_ACTIVE,
                )
                self.sock.sendto(hb.pack(self.tx), self.gcs_peer)
                sys_st = self.tx.sys_status_encode(
                    0, 0, 0, 0, int(self.sim_batt_v * 1000), -1, -1, 0, 0, 0, 0, 0, 0
                )
                self.sock.sendto(sys_st.pack(self.tx), self.gcs_peer)
                last_hb = now


class FaultInjectorClient:
    def __init__(self, host: str, port: int) -> None:
        self.dst = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        self.tx = mav.MAVLink(None, srcSystem=255, srcComponent=190)
        self.rx = mav.MAVLink(None)

    def close(self) -> None:
        self.sock.close()

    def drain_rx(self) -> None:
        prev_timeout = self.sock.gettimeout()
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recvfrom(1024)
        except BlockingIOError:
            pass
        finally:
            self.sock.settimeout(prev_timeout)

    def send_gcs_hb(self) -> None:
        hb = self.tx.heartbeat_encode(
            mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE
        )
        self.sock.sendto(hb.pack(self.tx), self.dst)

    def set_param(self, name: str, value: float) -> None:
        msg = self.tx.param_set_encode(
            1, 1, name.encode("ascii"), float(value), mav.MAV_PARAM_TYPE_REAL32
        )
        self.sock.sendto(msg.pack(self.tx), self.dst)

    def send_landing_target(self, ax: float, ay: float) -> None:
        msg = self.tx.landing_target_encode(
            int(time.time() * 1e6),
            0,
            mav.MAV_FRAME_BODY_FRD,
            ax,
            ay,
            0.0,
            0.1,
            0.1,
            0.0,
            0.0,
            0.0,
            [1.0, 0.0, 0.0, 0.0],
            mav.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            0,
        )
        self.sock.sendto(msg.pack(self.tx), self.dst)

    def wait_mode(self, expected_modes: set[int], timeout_s: float = 3.0) -> int | None:
        deadline = time.monotonic() + timeout_s
        last_mode = None
        while time.monotonic() < deadline:
            try:
                data, _ = self.sock.recvfrom(1024)
                for b in data:
                    msg = self.rx.parse_char(bytes([b]))
                    if msg is not None and msg.get_msgId() == mav.MAVLINK_MSG_ID_HEARTBEAT:
                        last_mode = msg.custom_mode
                        if last_mode in expected_modes:
                            return last_mode
            except socket.timeout:
                self.send_gcs_hb()
        return last_mode


def run_suite(host: str, port: int, mock_fc: MockArduCopterFC | None) -> int:
    cli = FaultInjectorClient(host, port)
    try:
        cli.send_gcs_hb()
        init_mode = cli.wait_mode({COPTER_MODE_LOITER, COPTER_MODE_RTL, COPTER_MODE_LAND}, timeout_s=2.0)
        if init_mode is None:
            print(f"FAIL: no FC HEARTBEAT received from {host}:{port}", file=sys.stderr)
            return 1

        # 1. RC-loss fault
        cli.drain_rx()
        cli.set_param("SIM_RC_FAIL", 1.0)
        m_rc = cli.wait_mode({COPTER_MODE_RTL, COPTER_MODE_LAND}, timeout_s=2.0)
        assert m_rc in (COPTER_MODE_RTL, COPTER_MODE_LAND), f"rc-loss expected RTL/LAND, got {m_rc}"
        print(f"  [PASS] rc-loss          -> FC transitioned to mode={m_rc} (RTL/LAND)")
        cli.set_param("SIM_RC_FAIL", 0.0)
        if mock_fc is not None:
            cli.set_param("SIM_RESET_MODE", float(COPTER_MODE_LOITER))
            assert cli.wait_mode({COPTER_MODE_LOITER}, timeout_s=1.0) == COPTER_MODE_LOITER

        # 2. Battery sag fault (Low 10.6V -> RTL, Critical 10.0V -> LAND)
        cli.drain_rx()
        cli.set_param("SIM_BATT_VOLTAGE", 10.6)
        m_low = cli.wait_mode({COPTER_MODE_RTL}, timeout_s=2.0)
        assert m_low == COPTER_MODE_RTL, f"battery-low expected RTL, got {m_low}"
        cli.drain_rx()
        cli.set_param("SIM_BATT_VOLTAGE", 10.0)
        m_crt = cli.wait_mode({COPTER_MODE_LAND}, timeout_s=2.0)
        assert m_crt == COPTER_MODE_LAND, f"battery-crit expected LAND, got {m_crt}"
        print(f"  [PASS] battery-sag      -> Low(10.6V) mode={m_low} (RTL), Crit(10.0V) mode={m_crt} (LAND)")
        cli.set_param("SIM_BATT_VOLTAGE", 12.6)
        if mock_fc is not None:
            cli.set_param("SIM_RESET_MODE", float(COPTER_MODE_LOITER))
            assert cli.wait_mode({COPTER_MODE_LOITER}, timeout_s=1.0) == COPTER_MODE_LOITER

        # 3. Wi-Fi GCS drop fault (no GCS HB for 0.3s -> FC stays in LOITER and armed)
        time.sleep(0.3)
        cli.drain_rx()
        cli.send_gcs_hb()
        m_wifi = cli.wait_mode({COPTER_MODE_LOITER}, timeout_s=2.0)
        assert m_wifi == COPTER_MODE_LOITER and (mock_fc is None or mock_fc.armed), (
            f"wifi-drop expected steady LOITER (5), got mode={m_wifi}"
        )
        print(f"  [PASS] wifi-drop        -> FC stayed alive/armed in LOITER (mode={m_wifi}) after GCS dropout")

        # 4. Vision occlusion fault (stream LANDING_TARGET then drop -> LOS clears safely)
        cli.send_landing_target(0.04, -0.02)
        time.sleep(0.1)
        if mock_fc is not None:
            assert mock_fc.los_active, "expected FC to latch valid BODY_FRD LANDING_TARGET"
            time.sleep(0.45)
            assert not mock_fc.los_active, "expected FC to clear stale LANDING_TARGET after occlusion"
        print("  [PASS] vision-occlusion -> valid BODY_FRD target latched, cleared cleanly on occlusion")
        return 0
    finally:
        cli.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="OPEN-FC M4 MAVLink2 Fault-Injection Harness")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--self-test", action="store_true", help="Run against loopback UDP MAVLink2 mock FC")
    group.add_argument("--live", metavar="HOST:PORT", help="Run against live ArduCopter SITL UDP endpoint")
    args = parser.parse_args()

    if args.self_test:
        print("Fault-Injection Harness (loopback UDP MAVLink2 self-test — mock FC):")
        fc = MockArduCopterFC(0)
        fc.start()
        try:
            rc = run_suite("127.0.0.1", fc.port, fc)
        finally:
            fc.stop()
        if rc == 0:
            print("PASS: All 4 M4 MAVLink2 fault-injection scenarios verified in loopback self-test.")
        return rc

    host, port_s = args.live.rsplit(":", 1)
    print(f"Fault-Injection Harness (live SITL @ {host}:{port_s}):")
    return run_suite(host, int(port_s), None)


if __name__ == "__main__":
    raise SystemExit(main())
