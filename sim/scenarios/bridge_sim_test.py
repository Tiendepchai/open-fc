#!/usr/bin/env python3
"""Loopback UDP & Stream Verification for esp32-wifi MAVLink1/2 Bridge (M2/M4).

Mirrors exact C logic in firmware/esp32-wifi/main/{mavlink_framer.c,bridge_main.c}:
  1. Byte-by-byte fragmented MAVLink1 (0xFE) & MAVLink2 (0xFD) + max-size packet framing
  2. Signed MAVLink2 (+13B signature when incompat_flags bit 0 is set)
  3. Garbage prefix resync & UART framing-error reset recovery
  4. Invalid MAVLink2 incompat_flags reset & immediate resync
  5. Concatenated multi-packet UDP datagrams (v2+v2, v1+v2, unsigned+signed)
  6. Malformed UDP datagram rejection & peer-hijack protection
  7. Temporary UDP peer disconnect & reconnect onto a new ephemeral port

All timing/throughput behavior is host Python loopback simulation ONLY.
"""

from __future__ import annotations

from dataclasses import dataclass
import queue
import socket
import sys
import threading
import time

from pymavlink.dialects.v10 import ardupilotmega as mav1
from pymavlink.dialects.v20 import ardupilotmega as mav2

OPENFC_MAVLINK_MAX_PACKET_LEN = 280
OPENFC_MAVLINK_STX_V1 = 0xFE
OPENFC_MAVLINK_STX_V2 = 0xFD
OPENFC_MAVLINK_V2_IFLAG_SIGNED = 0x01
UDP_RX_BUF_SIZE = 1500


@dataclass
class FramerStats:
    """Exact mirror of openfc_mavlink_framer_stats_t."""

    packets_v1: int = 0
    packets_v2: int = 0
    bytes_dropped_resync: int = 0
    framing_resets: int = 0


def _compute_v1_len(payload_len: int) -> int:
    return 8 + payload_len


def _compute_v2_len(payload_len: int, incompat_flags: int) -> int:
    sig = 13 if (incompat_flags & OPENFC_MAVLINK_V2_IFLAG_SIGNED) != 0 else 0
    return 12 + payload_len + sig


class MavlinkFramer:
    """Exact mirror of openfc_mavlink_framer_t and openfc_mavlink_framer_feed() in mavlink_framer.c."""

    def __init__(self) -> None:
        self.buf = bytearray(OPENFC_MAVLINK_MAX_PACKET_LEN)
        self.pos = 0
        self.expected_len = 0
        self.stats = FramerStats()

    def reset(self) -> None:
        if self.pos > 0:
            self.stats.framing_resets += 1
            self.stats.bytes_dropped_resync += self.pos
        self.pos = 0
        self.expected_len = 0

    def _rescan_rejected_v2_header(self) -> None:
        """Exact mirror of rescan_rejected_v2_header() in mavlink_framer.c:40-66."""
        b1 = self.buf[1]
        b2 = self.buf[2]
        self.stats.framing_resets += 1
        self.stats.bytes_dropped_resync += 1  # drop buf[0]
        self.pos = 0
        self.expected_len = 0

        if b1 == OPENFC_MAVLINK_STX_V1:
            self.buf[0] = b1
            self.buf[1] = b2
            self.pos = 2
            self.expected_len = _compute_v1_len(b2)
        elif b1 == OPENFC_MAVLINK_STX_V2:
            self.buf[0] = b1
            self.buf[1] = b2
            self.pos = 2
        else:
            self.stats.bytes_dropped_resync += 1  # drop b1
            if b2 in (OPENFC_MAVLINK_STX_V1, OPENFC_MAVLINK_STX_V2):
                self.buf[0] = b2
                self.pos = 1
            else:
                self.stats.bytes_dropped_resync += 1  # drop b2

    def feed(self, byte: int) -> bytes | None:
        byte &= 0xFF

        if self.pos >= OPENFC_MAVLINK_MAX_PACKET_LEN or (
            self.pos > 0 and self.buf[0] not in (OPENFC_MAVLINK_STX_V1, OPENFC_MAVLINK_STX_V2)
        ):
            self.reset()

        if self.pos == 0:
            if byte in (OPENFC_MAVLINK_STX_V2, OPENFC_MAVLINK_STX_V1):
                self.buf[0] = byte
                self.pos = 1
                self.expected_len = 0
            else:
                self.stats.bytes_dropped_resync += 1
            return None

        self.buf[self.pos] = byte
        self.pos += 1

        if self.buf[0] == OPENFC_MAVLINK_STX_V1:
            if self.pos == 2:
                self.expected_len = _compute_v1_len(self.buf[1])
        else:
            if self.pos == 3:
                incompat = self.buf[2]
                if (incompat & (~OPENFC_MAVLINK_V2_IFLAG_SIGNED & 0xFF)) != 0:
                    self._rescan_rejected_v2_header()
                    return None
                self.expected_len = _compute_v2_len(self.buf[1], incompat)

        if self.expected_len > OPENFC_MAVLINK_MAX_PACKET_LEN:
            self.reset()
            return None

        if self.expected_len > 0 and self.pos == self.expected_len:
            pkt = bytes(self.buf[: self.expected_len])
            if self.buf[0] == OPENFC_MAVLINK_STX_V2:
                self.stats.packets_v2 += 1
            else:
                self.stats.packets_v1 += 1
            self.pos = 0
            self.expected_len = 0
            return pkt

        return None


def validate_mavlink_datagram(data: bytes) -> bool:
    """Exact mirror of openfc_mavlink_validate_datagram() in mavlink_framer.c."""
    length = len(data)
    if length == 0:
        return False
    offset = 0
    while offset < length:
        stx = data[offset]
        if stx == OPENFC_MAVLINK_STX_V1:
            if length - offset < 8:
                return False
            pkt_len = _compute_v1_len(data[offset + 1])
        elif stx == OPENFC_MAVLINK_STX_V2:
            if length - offset < 12:
                return False
            incompat = data[offset + 2]
            if (incompat & (~OPENFC_MAVLINK_V2_IFLAG_SIGNED & 0xFF)) != 0:
                return False
            pkt_len = _compute_v2_len(data[offset + 1], incompat)
        else:
            return False

        if pkt_len == 0 or pkt_len > OPENFC_MAVLINK_MAX_PACKET_LEN or pkt_len > (length - offset):
            return False
        offset += pkt_len
    return offset == length


class LoopbackWifiBridge:
    """Loopback UDP <-> simulated FC UART bridge mirroring esp32-wifi/main/bridge_main.c."""

    def __init__(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.02)
        self.port = self.sock.getsockname()[1]

        self.framer = MavlinkFramer()
        self._lock = threading.Lock()
        self.peer_addr: tuple[str, int] | None = None
        self.peer_learned = False
        self.uart_err_count = 0
        self.udp_tx_drop_count = 0
        self.udp_rx_reject_count = 0
        self.uart_rx_queue: queue.Queue[bytes] = queue.Queue()

        self._stop_evt = threading.Event()
        self._udp_thread = threading.Thread(target=self._udp_to_uart_loop, daemon=True)

    def start(self) -> None:
        self._udp_thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        self._udp_thread.join(timeout=1.0)
        self.sock.close()

    def _udp_to_uart_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                data, src = self.sock.recvfrom(UDP_RX_BUF_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                continue
            if not validate_mavlink_datagram(data):
                with self._lock:
                    self.udp_rx_reject_count += 1
                continue
            with self._lock:
                self.peer_addr = src
                self.peer_learned = True
            self.uart_rx_queue.put(data)

    def feed_fc_uart_bytes(self, raw: bytes, chunk_size: int = 1) -> int:
        """Feed bytes from simulated FC UART into MavlinkFramer and send framed packets over UDP."""
        sent_packets = 0
        for off in range(0, len(raw), chunk_size):
            chunk = raw[off : off + chunk_size]
            for b in chunk:
                pkt = self.framer.feed(b)
                if pkt is None:
                    continue
                with self._lock:
                    dst = self.peer_addr
                if dst is not None:
                    try:
                        self.sock.sendto(pkt, dst)
                        sent_packets += 1
                    except OSError:
                        with self._lock:
                            self.udp_tx_drop_count += 1
        return sent_packets

    def trigger_uart_frame_error(self) -> None:
        """Mirror bridge_main.c UART_FRAME_ERR / UART_FIFO_OVF recovery path."""
        self.uart_err_count += 1
        self.framer.reset()


def _make_packets() -> dict[str, bytes]:
    tx1 = mav1.MAVLink(None, srcSystem=1, srcComponent=1)
    v1_hb = tx1.heartbeat_encode(
        mav1.MAV_TYPE_QUADROTOR, mav1.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 5, mav1.MAV_STATE_ACTIVE
    ).pack(tx1)

    tx2 = mav2.MAVLink(None, srcSystem=1, srcComponent=1)
    v2_hb = tx2.heartbeat_encode(
        mav2.MAV_TYPE_QUADROTOR, mav2.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 5, mav2.MAV_STATE_ACTIVE
    ).pack(tx2)
    v2_lt = tx2.landing_target_encode(
        1_234_567,
        0,
        mav2.MAV_FRAME_BODY_FRD,
        0.04,
        -0.02,
        0.0,
        0.1,
        0.1,
        0.0,
        0.0,
        0.0,
        [1.0, 0.0, 0.0, 0.0],
        mav2.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        0,
    ).pack(tx2)

    tx2_sig = mav2.MAVLink(None, srcSystem=1, srcComponent=1)
    tx2_sig.signing.secret_key = bytes(range(32))
    tx2_sig.signing.link_id = 1
    tx2_sig.signing.timestamp = 10_000
    tx2_sig.signing.sign_outgoing = True
    v2_signed_hb = tx2_sig.heartbeat_encode(
        mav2.MAV_TYPE_QUADROTOR, mav2.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 5, mav2.MAV_STATE_ACTIVE
    ).pack(tx2_sig)
    v2_signed_lt = tx2_sig.landing_target_encode(
        2_345_678,
        0,
        mav2.MAV_FRAME_BODY_FRD,
        -0.03,
        0.05,
        0.0,
        0.12,
        0.12,
        0.0,
        0.0,
        0.0,
        [1.0, 0.0, 0.0, 0.0],
        mav2.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        0,
    ).pack(tx2_sig)

    tx_gcs = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
    gcs_hb = tx_gcs.heartbeat_encode(
        mav2.MAV_TYPE_GCS, mav2.MAV_AUTOPILOT_INVALID, 0, 0, mav2.MAV_STATE_ACTIVE
    ).pack(tx_gcs)

    return {
        "v1_hb": v1_hb,
        "v2_hb": v2_hb,
        "v2_lt": v2_lt,
        "v2_signed_hb": v2_signed_hb,
        "v2_signed_lt": v2_signed_lt,
        "gcs_hb": gcs_hb,
    }


def _decode_v1(raw: bytes) -> object:
    rx = mav1.MAVLink(None)
    parsed = None
    for b in raw:
        m = rx.parse_char(bytes([b]))
        if m is not None:
            parsed = m
    assert parsed is not None, "failed to decode MAVLink1 packet via pymavlink"
    return parsed


def _decode_v2(raw: bytes) -> object:
    rx = mav2.MAVLink(None)
    parsed = None
    for b in raw:
        m = rx.parse_char(bytes([b]))
        if m is not None:
            parsed = m
    assert parsed is not None, "failed to decode MAVLink2 packet via pymavlink"
    return parsed


def _recv_exact(sock: socket.socket, timeout_s: float = 1.0) -> bytes:
    sock.settimeout(timeout_s)
    data, _ = sock.recvfrom(UDP_RX_BUF_SIZE)
    return data


def _register_gcs(gcs_sock: socket.socket, bridge: LoopbackWifiBridge, gcs_hb: bytes) -> None:
    gcs_sock.sendto(gcs_hb, ("127.0.0.1", bridge.port))
    forwarded = bridge.uart_rx_queue.get(timeout=1.0)
    assert forwarded == gcs_hb, "GCS registration HEARTBEAT did not reach FC UART intact"


def run_tests() -> int:
    pkts = _make_packets()
    assert pkts["v1_hb"][0] == OPENFC_MAVLINK_STX_V1 and len(pkts["v1_hb"]) == 8 + pkts["v1_hb"][1]
    assert pkts["v2_hb"][0] == OPENFC_MAVLINK_STX_V2 and len(pkts["v2_hb"]) == 12 + pkts["v2_hb"][1]
    assert pkts["v2_signed_hb"][2] == OPENFC_MAVLINK_V2_IFLAG_SIGNED
    assert len(pkts["v2_signed_hb"]) == 12 + pkts["v2_signed_hb"][1] + 13
    assert len(pkts["v2_signed_lt"]) == 12 + pkts["v2_signed_lt"][1] + 13

    bridge = LoopbackWifiBridge()
    bridge.start()
    gcs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gcs.bind(("127.0.0.1", 0))
    gcs_port_1 = gcs.getsockname()[1]

    try:
        _register_gcs(gcs, bridge, pkts["gcs_hb"])
        assert bridge.peer_learned and bridge.peer_addr == ("127.0.0.1", gcs_port_1)

        # 1. Byte-by-byte fragmented MAVLink1 & MAVLink2 + irregular chunking + embedded STX + max-size packets
        assert bridge.feed_fc_uart_bytes(pkts["v1_hb"], chunk_size=1) == 1
        rx_v1 = _recv_exact(gcs)
        assert rx_v1 == pkts["v1_hb"]
        assert _decode_v1(rx_v1).get_msgId() == mav1.MAVLINK_MSG_ID_HEARTBEAT

        assert bridge.feed_fc_uart_bytes(pkts["v2_hb"], chunk_size=1) == 1
        rx_v2_hb = _recv_exact(gcs)
        assert rx_v2_hb == pkts["v2_hb"]
        assert _decode_v2(rx_v2_hb).get_msgId() == mav2.MAVLINK_MSG_ID_HEARTBEAT

        assert bridge.feed_fc_uart_bytes(pkts["v2_lt"], chunk_size=3) == 1
        rx_v2_lt = _recv_exact(gcs)
        assert rx_v2_lt == pkts["v2_lt"]
        m_lt = _decode_v2(rx_v2_lt)
        assert m_lt.get_msgId() == mav2.MAVLINK_MSG_ID_LANDING_TARGET
        assert m_lt.frame == mav2.MAV_FRAME_BODY_FRD

        # Packet containing literal STX bytes (0xFD / 0xFE) inside payload must not trigger false split/reset
        stx_payload = bytes([0xFD, 0xFE, 0xFD, 0xFE, 0x01, 0x02, 0xFD, 0xFE])
        stx_in_v2 = bytes([OPENFC_MAVLINK_STX_V2, len(stx_payload), 0x00, 0x00, 0x07, 0x01, 0x01, 0x00, 0x00, 0x00]) + stx_payload + bytes([0x12, 0x34])
        assert bridge.feed_fc_uart_bytes(stx_in_v2, chunk_size=1) == 1
        assert _recv_exact(gcs) == stx_in_v2

        # Max-size boundary packets: 263B v1 (len=255), 267B v2 unsigned, 280B v2 signed
        max_v1 = bytes([OPENFC_MAVLINK_STX_V1, 255]) + bytes(261)
        max_v2_u = bytes([OPENFC_MAVLINK_STX_V2, 255, 0x00]) + bytes(264)
        max_v2_s = bytes([OPENFC_MAVLINK_STX_V2, 255, OPENFC_MAVLINK_V2_IFLAG_SIGNED]) + bytes(277)
        assert len(max_v1) == 263 and len(max_v2_u) == 267 and len(max_v2_s) == 280
        for raw_max in (max_v1, max_v2_u, max_v2_s):
            assert bridge.feed_fc_uart_bytes(raw_max, chunk_size=1) == 1
            assert _recv_exact(gcs) == raw_max
        print("  [PASS] byte_by_byte_v1_v2        : MAVLink1/2 fragmented + embedded STX + max-size (263B/267B/280B) framed intact")

        # 2. Signed MAVLink2 (+13B signature)
        v2_before = bridge.framer.stats.packets_v2
        for signed_pkt in (pkts["v2_signed_hb"], pkts["v2_signed_lt"]):
            assert bridge.feed_fc_uart_bytes(signed_pkt, chunk_size=1) == 1
            got = _recv_exact(gcs)
            assert got == signed_pkt
            assert validate_mavlink_datagram(got)
        assert bridge.framer.stats.packets_v2 == v2_before + 2
        print("  [PASS] signed_mavlink2_13b       : signed HEARTBEAT (34B) & LANDING_TARGET (+13B sig) framed & validated")

        # 3. Garbage prefix resync + UART frame-error reset recovery
        drop_before = bridge.framer.stats.bytes_dropped_resync
        reset_before = bridge.framer.stats.framing_resets
        garbage_a = bytes([0x00, 0xAA, 0x55, 0x12, 0x34, 0x7F, 0x80, 0x99, 0xFF])  # 9 non-STX bytes
        garbage_b = bytes([0x11, 0x22, 0x33, 0x44, 0x55])                           # 5 non-STX bytes
        assert bridge.feed_fc_uart_bytes(garbage_a + pkts["v1_hb"] + garbage_b + pkts["v2_hb"], chunk_size=1) == 2
        assert _recv_exact(gcs) == pkts["v1_hb"]
        assert _recv_exact(gcs) == pkts["v2_hb"]
        assert bridge.framer.stats.bytes_dropped_resync == drop_before + len(garbage_a) + len(garbage_b)

        # Partial packet interrupted by UART_FRAME_ERR -> reset -> drop-byte accounting & clean resync
        drop_before_err = bridge.framer.stats.bytes_dropped_resync
        assert bridge.feed_fc_uart_bytes(pkts["v2_hb"][:6], chunk_size=1) == 0
        assert bridge.framer.pos == 6
        bridge.trigger_uart_frame_error()
        assert bridge.framer.pos == 0 and bridge.framer.expected_len == 0
        assert bridge.framer.stats.framing_resets == reset_before + 1
        assert bridge.framer.stats.bytes_dropped_resync == drop_before_err + 6
        # Resetting an already-idle framer must be a no-op
        bridge.framer.reset()
        assert bridge.framer.stats.framing_resets == reset_before + 1
        assert bridge.framer.stats.bytes_dropped_resync == drop_before_err + 6
        assert bridge.feed_fc_uart_bytes(pkts["v2_hb"], chunk_size=1) == 1
        assert _recv_exact(gcs) == pkts["v2_hb"]
        print(
            "  [PASS] garbage_prefix_resync     : 14 garbage bytes dropped + mid-packet UART reset (+6 drop bytes) recovered cleanly"
        )

        # 4. Invalid incompat_flags reset (0x80, 0x02, 0x04) + embedded-STX V2 header rescan paths
        resets_before = bridge.framer.stats.framing_resets
        drops_before = bridge.framer.stats.bytes_dropped_resync
        for bad_flag in (0x80, 0x02, 0x04):
            bad_hdr = bytes([OPENFC_MAVLINK_STX_V2, 9, bad_flag])
            assert bridge.feed_fc_uart_bytes(bad_hdr, chunk_size=1) == 0
            assert bridge.framer.pos == 0 and bridge.framer.expected_len == 0
        assert bridge.framer.stats.framing_resets == resets_before + 3
        assert bridge.framer.stats.bytes_dropped_resync == drops_before + 9
        assert bridge.feed_fc_uart_bytes(pkts["v2_lt"], chunk_size=1) == 1
        assert _recv_exact(gcs) == pkts["v2_lt"]

        # Mirror mavlink_framer.c rescan_rejected_v2_header() across all 5 embedded-STX patterns
        rescan_cases = [
            ("fd+v2", bytes([0xFD]), pkts["v2_hb"], 1),
            ("fd_fd+v2", bytes([0xFD, 0xFD]), pkts["v2_hb"], 2),
            ("fd_55+v2", bytes([0xFD, 0x55]), pkts["v2_hb"], 2),
            ("fd+v1", bytes([0xFD]), pkts["v1_hb"], 1),
            ("fd_55+v1", bytes([0xFD, 0x55]), pkts["v1_hb"], 2),
        ]
        for label, prefix, target_pkt, expect_drops in rescan_cases:
            d0 = bridge.framer.stats.bytes_dropped_resync
            assert bridge.feed_fc_uart_bytes(prefix + target_pkt, chunk_size=1) == 1, f"rescan {label} failed"
            assert _recv_exact(gcs) == target_pkt, f"rescan {label} packet mismatch"
            assert bridge.framer.stats.bytes_dropped_resync == d0 + expect_drops, (
                f"rescan {label}: expected +{expect_drops} drops, got {bridge.framer.stats.bytes_dropped_resync - d0}"
            )
        print(
            "  [PASS] invalid_incompat_rescan   : 3 invalid incompat_flags + 5 embedded-STX header rescan paths verified"
        )

        # 5. Concatenated UDP datagrams (v2+v2, v1+v2, unsigned+signed, and >1024B coalesced MTU datagram)
        coalesced_1192b = (
            pkts["v1_hb"]
            + pkts["v2_hb"]
            + pkts["v2_signed_hb"]
            + (max_v2_s * 4)
        )
        assert len(coalesced_1192b) == 1192 and 1024 < len(coalesced_1192b) <= UDP_RX_BUF_SIZE
        concat_cases = [
            ("v2+v2", pkts["v2_hb"] + pkts["v2_lt"], [pkts["v2_hb"], pkts["v2_lt"]]),
            ("v1+v2", pkts["v1_hb"] + pkts["v2_hb"], [pkts["v1_hb"], pkts["v2_hb"]]),
            ("v2+v2_signed", pkts["v2_hb"] + pkts["v2_signed_hb"], [pkts["v2_hb"], pkts["v2_signed_hb"]]),
            (
                "coalesced_1192B",
                coalesced_1192b,
                [pkts["v1_hb"], pkts["v2_hb"], pkts["v2_signed_hb"], max_v2_s, max_v2_s, max_v2_s, max_v2_s],
            ),
        ]
        for label, dgram, expected_parts in concat_cases:
            assert validate_mavlink_datagram(dgram), f"concat {label} should be valid"
            gcs.sendto(dgram, ("127.0.0.1", bridge.port))
            got_uart = bridge.uart_rx_queue.get(timeout=1.0)
            assert got_uart == dgram, f"concat {label} altered on FC UART"
            # Re-frame on FC side to verify every packet extracts cleanly
            fc_framer = MavlinkFramer()
            extracted = [p for b in got_uart if (p := fc_framer.feed(b)) is not None]
            assert extracted == expected_parts, f"concat {label} re-frame mismatch"
        print(
            "  [PASS] concatenated_datagrams    : v2+v2, v1+v2, unsigned+signed, and 1192B (>1024B) coalesced UDP forwarded intact"
        )

        # 6. Malformed UDP datagram rejection + peer-hijack protection
        assert not validate_mavlink_datagram(b""), "empty datagram must be rejected"
        malformed_dgrams = [
            b"garbage-udp-noise",
            pkts["v1_hb"][:7],                                                  # short v1 (< 8B)
            pkts["v1_hb"][:-1],                                                 # truncated v1
            pkts["v2_hb"][:11],                                                 # short v2 (< 12B)
            pkts["v2_hb"][:-1],                                                 # truncated v2 unsigned
            pkts["v2_signed_hb"][:-1],                                          # truncated v2 signed (missing 1B sig)
            bytes([OPENFC_MAVLINK_STX_V2, 9, 0x02]) + bytes(18),                # invalid incompat 0x02
            bytes([OPENFC_MAVLINK_STX_V2, 9, 0x80]) + bytes(18),                # invalid incompat 0x80
            pkts["v2_hb"] + b"\x99",                                            # valid pkt + trailing garbage
            pkts["v2_hb"] + pkts["v2_lt"][:-1],                                 # concat with truncated 2nd pkt
            bytes([OPENFC_MAVLINK_STX_V2, 255, 0x00]) + bytes(20),              # declared len > datagram len
        ]
        rogue = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rogue.bind(("127.0.0.1", 0))
        try:
            reject_before = bridge.udp_rx_reject_count
            for bad in malformed_dgrams:
                assert not validate_mavlink_datagram(bad)
                rogue.sendto(bad, ("127.0.0.1", bridge.port))
            # Wait briefly for udp_to_uart loop to process every malformed datagram
            deadline = time.monotonic() + 1.5
            while bridge.udp_rx_reject_count < reject_before + len(malformed_dgrams) and time.monotonic() < deadline:
                time.sleep(0.01)
            assert bridge.udp_rx_reject_count == reject_before + len(malformed_dgrams), (
                f"expected {len(malformed_dgrams)} UDP rejects, got {bridge.udp_rx_reject_count - reject_before}"
            )
            assert bridge.uart_rx_queue.empty(), "malformed UDP bytes leaked to FC UART!"
            assert bridge.peer_addr == ("127.0.0.1", gcs_port_1), "malformed UDP hijacked peer_addr!"
        finally:
            rogue.close()
        print(
            f"  [PASS] malformed_udp_rejection   : {len(malformed_dgrams)}/{len(malformed_dgrams)} malformed "
            "datagrams + empty rejected, 0 UART leaks, peer_addr preserved"
        )

        # 7. Temporary UDP peer disconnect & reconnect on a new ephemeral port
        gcs.close()
        gcs = None
        v2_before_drop = bridge.framer.stats.packets_v2
        # Stream 5 packets while GCS #1 is disconnected — bridge must keep framing without stalling
        for _ in range(5):
            bridge.feed_fc_uart_bytes(pkts["v2_hb"], chunk_size=1)
        assert bridge.framer.stats.packets_v2 == v2_before_drop + 5

        # Reconnect GCS #2 on a guaranteed-distinct ephemeral port P2 != P1
        held_socks: list[socket.socket] = []
        gcs2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        gcs2.bind(("127.0.0.1", 0))
        while gcs2.getsockname()[1] == gcs_port_1:
            held_socks.append(gcs2)
            gcs2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            gcs2.bind(("127.0.0.1", 0))
        gcs_port_2 = gcs2.getsockname()[1]
        assert gcs_port_2 != gcs_port_1
        try:
            _register_gcs(gcs2, bridge, pkts["gcs_hb"])
            assert bridge.peer_addr == ("127.0.0.1", gcs_port_2), "bridge failed to re-learn new GCS peer"
            assert bridge.feed_fc_uart_bytes(pkts["v2_lt"], chunk_size=1) == 1
            rx_after_reconnect = _recv_exact(gcs2)
            assert rx_after_reconnect == pkts["v2_lt"]
            decoded = _decode_v2(rx_after_reconnect)
            assert decoded.get_msgId() == mav2.MAVLINK_MSG_ID_LANDING_TARGET
        finally:
            gcs2.close()
            for hs in held_socks:
                hs.close()
        assert bridge.framer.stats.packets_v1 == 5, f"unexpected v1 count: {bridge.framer.stats.packets_v1}"
        assert bridge.framer.stats.packets_v2 == 19, f"unexpected v2 count: {bridge.framer.stats.packets_v2}"
        print(
            f"  [PASS] udp_peer_reconnect        : survived peer drop (5 pkts streamed during outage) & "
            f"re-learned new GCS port ({gcs_port_1} -> {gcs_port_2}); "
            f"framer totals v1={bridge.framer.stats.packets_v1}, v2={bridge.framer.stats.packets_v2}"
        )

        print("PASS: All 7 Wi-Fi MAVLink1/2 bridge framer & loopback UDP simulation tests verified.")
        return 0
    finally:
        if gcs is not None:
            gcs.close()
        bridge.stop()


if __name__ == "__main__":
    raise SystemExit(run_tests())
