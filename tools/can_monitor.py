#!/usr/bin/env python3
"""
Realtime SocketCAN node monitor for MuJoCo virtual CAN bridge.

Features:
  - Passive CAN sniffing on vcan0/can0.
  - Optional periodic E2Box IMU polling: 0x221#03.
  - E2Box IMU decode:
      0x2A1 quaternion frame
      0x321 gyro frame
  - SPG/MIT actuator decode:
      configured CAN IDs, default 0x141..0x14C
      0xC0 MIT status
      0xC1 enter ACK
      0xC2 exit ACK
      0xC3 set-zero ACK
  - CAN bus load estimation:
      sliding-window rx/tx frame rate
      estimated bus bit rate
      estimated bus utilization percentage
      terminal bar graph
  - Terminal dashboard using curses.

No external Python package is required.
Linux SocketCAN only.

Bus-load note:
  This is an estimate based on observed SocketCAN frames and a classical CAN
  bit-count model. It is good for relative load/debugging, but it is not a
  substitute for a logic analyzer/CAN analyzer when exact physical bus timing,
  error frames, arbitration loss, retransmissions, or bit-stuffing details matter.
"""

from __future__ import annotations

import argparse
import curses
import math
import select
import signal
import socket
import struct
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional, Tuple


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

E2BOX_REQ_ID = 0x221
E2BOX_QUAT_ID = 0x2A1
E2BOX_GYRO_ID = 0x321
E2BOX_CMD_GET_ALL = 0x03

SPG_CMD_MIT_CONTROL = 0xC0
SPG_CMD_MIT_ENTER = 0xC1
SPG_CMD_MIT_EXIT = 0xC2
SPG_CMD_MIT_SET_ZERO = 0xC3
SPG_CMD_CLEAR_ERROR = 0x9B


def monotonic() -> float:
    return time.monotonic()


def fmt_age(last_t: float, now: float) -> str:
    if last_t <= 0.0:
        return "never"
    age = now - last_t
    if age < 1.0:
        return f"{age * 1000.0:6.1f} ms"
    return f"{age:6.2f} s "


def fmt_hex_data(data: bytes, dlc: int) -> str:
    return " ".join(f"{b:02X}" for b in data[:dlc])


def normalize_quat_xyzw(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12 or not math.isfinite(n):
        return 0.0, 0.0, 0.0, 1.0
    return qx / n, qy / n, qz / n, qw / n


def projected_gravity_from_xyzw(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    # Matches the user's Python E2BoxIMUProtocol convention.
    qx, qy, qz, qw = q

    vx, vy, vz = 0.0, 0.0, -1.0

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    vpx = vx - qw * tx + (qy * tz - qz * ty)
    vpy = vy - qw * ty + (qz * tx - qx * tz)
    vpz = vz - qw * tz + (qx * ty - qy * tx)

    return vpy, vpx, vpz


def estimate_classical_can_bits(
    dlc: int,
    *,
    is_eff: bool = False,
    is_rtr: bool = False,
    stuff_factor: float = 1.15,
    include_intermission: bool = True,
) -> int:
    """Estimate physical bits consumed by one classical CAN frame.

    Standard 11-bit data frame base length is approximated as:
      SOF 1
      arbitration 12
      control 6
      data 8*DLC
      CRC sequence/delimiter 16
      ACK slot/delimiter 2
      EOF 7
      intermission 3
    = 47 + 8*DLC bits.

    Extended 29-bit frame is approximated as:
      standard base + extra 20 arbitration/control bits
    = 67 + 8*DLC bits.

    Bit stuffing is protocol-dependent and data-dependent. For runtime load
    display we multiply by a configurable factor instead of trying to reproduce
    exact bit stuffing.
    """
    dlc = max(0, min(int(dlc), 8))
    data_bits = 0 if is_rtr else 8 * dlc

    if is_eff:
        base_bits = 67 + data_bits
    else:
        base_bits = 47 + data_bits

    if not include_intermission:
        base_bits -= 3

    return max(0, int(math.ceil(base_bits * stuff_factor)))


def make_bar(percent: float, width: int = 32) -> str:
    percent = max(0.0, min(percent, 999.0))
    filled = int(round(min(percent, 100.0) / 100.0 * width))
    return "[" + "#" * filled + "." * (width - filled) + "]"


@dataclass
class BusLoadWindow:
    window_s: float = 1.0
    bitrate: float = 1_000_000.0
    events: Deque[Tuple[float, int, str]] = field(default_factory=deque)

    def add(self, timestamp: float, bits: int, direction: str) -> None:
        self.events.append((timestamp, max(0, bits), direction))
        self.prune(timestamp)

    def prune(self, now: float) -> None:
        cutoff = now - self.window_s
        while self.events and self.events[0][0] < cutoff:
            self.events.popleft()

    def stats(self, now: float) -> Tuple[int, int, int, int, float, float]:
        self.prune(now)

        rx_frames = 0
        tx_frames = 0
        rx_bits = 0
        tx_bits = 0

        for _t, bits, direction in self.events:
            if direction == "rx":
                rx_frames += 1
                rx_bits += bits
            elif direction == "tx":
                tx_frames += 1
                tx_bits += bits

        duration = max(self.window_s, 1e-9)
        total_bits = rx_bits + tx_bits
        bit_rate = total_bits / duration
        load_percent = (bit_rate / max(self.bitrate, 1e-9)) * 100.0
        return rx_frames, tx_frames, rx_bits, tx_bits, bit_rate, load_percent


@dataclass
class RawFrameState:
    can_id: int
    rx_count: int = 0
    last_t: float = 0.0
    last_dlc: int = 0
    last_data: bytes = b"\x00" * 8


@dataclass
class E2BoxState:
    req_count: int = 0

    quat_count: int = 0
    quat_last_t: float = 0.0
    quat_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    projected_gravity_b: Tuple[float, float, float] = (0.0, 0.0, -1.0)

    gyro_count: int = 0
    gyro_last_t: float = 0.0
    angular_velocity_rad_s: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class SPGMotorState:
    can_id: int
    rx_count: int = 0
    last_t: float = 0.0
    last_opcode: Optional[int] = None
    last_kind: str = "none"

    enabled_hint: Optional[bool] = None
    temp_c: Optional[int] = None
    iq_count: Optional[int] = None
    iq_a_approx: Optional[float] = None
    speed_dps: Optional[int] = None
    position_rad: Optional[float] = None
    zero_offset_count: Optional[int] = None
    raw_data: bytes = b"\x00" * 8


@dataclass
class MonitorState:
    start_t: float = field(default_factory=monotonic)
    total_rx: int = 0
    total_tx: int = 0

    raw_frames: Dict[int, RawFrameState] = field(default_factory=dict)
    imu: E2BoxState = field(default_factory=E2BoxState)
    motors: Dict[int, SPGMotorState] = field(default_factory=dict)
    bus_load: BusLoadWindow = field(default_factory=BusLoadWindow)


def open_can_socket(iface: str) -> socket.socket:
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((iface,))
    sock.setblocking(False)
    return sock


def parse_can_frame(frame_bytes: bytes) -> Tuple[int, int, bytes, bool, bool, bool]:
    can_id_raw, dlc, data = struct.unpack(CAN_FRAME_FMT, frame_bytes)
    is_eff = bool(can_id_raw & CAN_EFF_FLAG)
    is_rtr = bool(can_id_raw & CAN_RTR_FLAG)
    is_err = bool(can_id_raw & CAN_ERR_FLAG)
    can_id = can_id_raw & (CAN_EFF_MASK if is_eff else CAN_SFF_MASK)
    dlc = min(int(dlc), 8)
    return can_id, dlc, data[:8], is_eff, is_rtr, is_err


def build_can_frame(can_id: int, data: bytes) -> bytes:
    if len(data) > 8:
        raise ValueError("Classical CAN payload must be <= 8 bytes")
    dlc = len(data)
    padded = data + b"\x00" * (8 - dlc)
    return struct.pack(CAN_FRAME_FMT, can_id & CAN_SFF_MASK, dlc, padded)


def send_can_frame(sock: socket.socket, can_id: int, data: bytes) -> None:
    sock.send(build_can_frame(can_id, data))


def parse_can_id_list(value: str) -> Tuple[int, ...]:
    ids = []
    for item in value.replace(";", ",").split(","):
        token = item.strip()
        if token:
            ids.append(int(token, 0))
    return tuple(ids)


def update_raw_state(state: MonitorState, can_id: int, dlc: int, data: bytes, now: float) -> None:
    raw = state.raw_frames.get(can_id)
    if raw is None:
        raw = RawFrameState(can_id=can_id)
        state.raw_frames[can_id] = raw
    raw.rx_count += 1
    raw.last_t = now
    raw.last_dlc = dlc
    raw.last_data = data


def decode_e2box_quat(data: bytes) -> Tuple[Tuple[float, float, float, float], Tuple[float, float, float]]:
    if len(data) < 8:
        raise ValueError("E2Box quaternion frame requires 8 bytes")

    qz_raw, qy_raw, qx_raw, qw_raw = struct.unpack("<hhhh", data[:8])

    qz = qz_raw / 10000.0
    qy = qy_raw / 10000.0
    qx = qx_raw / 10000.0
    qw = qw_raw / 10000.0

    # E2Box convention correction from host protocol.
    qx = -qx

    quat_xyzw = normalize_quat_xyzw((qx, qy, qz, qw))
    projected_gravity_b = projected_gravity_from_xyzw(quat_xyzw)
    return quat_xyzw, projected_gravity_b


def decode_e2box_gyro(data: bytes) -> Tuple[float, float, float]:
    if len(data) < 8:
        raise ValueError("E2Box gyro frame requires 8 bytes")

    gx_raw, gy_raw, gz_raw, _reserved = struct.unpack("<hhhh", data[:8])

    gx = (gx_raw / 100.0) * math.pi / 180.0
    gy = (gy_raw / 100.0) * math.pi / 180.0
    gz = (gz_raw / 100.0) * math.pi / 180.0

    # E2Box convention correction from host protocol.
    gx, gy = gy, gx
    return gx, gy, gz


def update_imu_state(state: MonitorState, can_id: int, dlc: int, data: bytes, now: float) -> None:
    if can_id == E2BOX_REQ_ID:
        state.imu.req_count += 1
        return

    if can_id == E2BOX_QUAT_ID and dlc == 8:
        try:
            quat, pg = decode_e2box_quat(data)
        except ValueError:
            return
        state.imu.quat_count += 1
        state.imu.quat_last_t = now
        state.imu.quat_xyzw = quat
        state.imu.projected_gravity_b = pg
        return

    if can_id == E2BOX_GYRO_ID and dlc == 8:
        try:
            gyro = decode_e2box_gyro(data)
        except ValueError:
            return
        state.imu.gyro_count += 1
        state.imu.gyro_last_t = now
        state.imu.angular_velocity_rad_s = gyro
        return


def decode_spg_frame(
    motor: SPGMotorState,
    dlc: int,
    data: bytes,
    now: float,
    tau_max_nm: float,
    feedback_pos_max_rad: float,
    iq_full_scale_count: float,
    iq_full_scale_current_a: float,
) -> None:
    motor.rx_count += 1
    motor.last_t = now
    motor.raw_data = data[:8]

    if dlc < 1:
        motor.last_opcode = None
        motor.last_kind = "empty"
        return

    opcode = data[0]
    motor.last_opcode = opcode

    if opcode == SPG_CMD_MIT_CONTROL and dlc == 8:
        temp = struct.unpack("<b", data[1:2])[0]
        iq_count = struct.unpack("<h", data[2:4])[0]
        speed_dps = struct.unpack("<h", data[4:6])[0]
        pos_i16 = struct.unpack("<h", data[6:8])[0]

        motor.last_kind = "MIT_STATUS"
        motor.temp_c = int(temp)
        motor.iq_count = int(iq_count)
        motor.iq_a_approx = (iq_count / iq_full_scale_count) * iq_full_scale_current_a
        motor.speed_dps = int(speed_dps)
        motor.position_rad = (pos_i16 / 32767.0) * feedback_pos_max_rad
        return

    if opcode == SPG_CMD_MIT_ENTER:
        motor.last_kind = "MIT_ENTER_ACK"
        motor.enabled_hint = True
        return

    if opcode == SPG_CMD_MIT_EXIT:
        motor.last_kind = "MIT_EXIT_ACK"
        motor.enabled_hint = False
        return

    if opcode == SPG_CMD_MIT_SET_ZERO:
        motor.last_kind = "MIT_SET_ZERO_ACK"
        if dlc == 8:
            motor.zero_offset_count = struct.unpack("<h", data[6:8])[0]
        return

    if opcode == SPG_CMD_CLEAR_ERROR:
        motor.last_kind = "CLEAR_ERROR_ACK"
        motor.enabled_hint = False
        return

    motor.last_kind = f"OP_{opcode:02X}"


def update_spg_state(
    state: MonitorState,
    can_id: int,
    dlc: int,
    data: bytes,
    now: float,
    spg_can_ids: Tuple[int, ...],
    tau_max_nm: float,
    feedback_pos_max_rad: float,
    iq_full_scale_count: float,
    iq_full_scale_current_a: float,
) -> None:
    if can_id not in spg_can_ids:
        return

    motor = state.motors.get(can_id)
    if motor is None:
        motor = SPGMotorState(can_id=can_id)
        state.motors[can_id] = motor

    decode_spg_frame(
        motor=motor,
        dlc=dlc,
        data=data,
        now=now,
        tau_max_nm=tau_max_nm,
        feedback_pos_max_rad=feedback_pos_max_rad,
        iq_full_scale_count=iq_full_scale_count,
        iq_full_scale_current_a=iq_full_scale_current_a,
    )


def handle_rx_frame(
    args: argparse.Namespace,
    state: MonitorState,
    can_id: int,
    dlc: int,
    data: bytes,
    is_eff: bool,
    is_rtr: bool,
) -> None:
    now = monotonic()
    state.total_rx += 1

    bits = estimate_classical_can_bits(
        dlc,
        is_eff=is_eff,
        is_rtr=is_rtr,
        stuff_factor=args.stuff_factor,
    )
    state.bus_load.add(now, bits, "rx")

    update_raw_state(state, can_id, dlc, data, now)
    update_imu_state(state, can_id, dlc, data, now)
    update_spg_state(
        state=state,
        can_id=can_id,
        dlc=dlc,
        data=data,
        now=now,
        spg_can_ids=args.spg_can_ids,
        tau_max_nm=args.tau_max_nm,
        feedback_pos_max_rad=args.feedback_pos_max_rad,
        iq_full_scale_count=args.iq_full_scale_count,
        iq_full_scale_current_a=args.iq_full_scale_current_a,
    )


def write_line(stdscr, row: int, text: str) -> int:
    h, w = stdscr.getmaxyx()
    if row >= h:
        return row + 1
    try:
        stdscr.addnstr(row, 0, text, max(0, w - 1))
    except curses.error:
        pass
    return row + 1


def render_dashboard(stdscr, args: argparse.Namespace, state: MonitorState) -> None:
    now = monotonic()
    elapsed = max(now - state.start_t, 1e-9)
    rx_rate = state.total_rx / elapsed

    rx_frames_w, tx_frames_w, rx_bits_w, tx_bits_w, bit_rate, load_percent = state.bus_load.stats(now)
    bar = make_bar(load_percent, width=args.bus_bar_width)

    stdscr.erase()
    row = 0

    row = write_line(stdscr, row, "MuJoCo Virtual CAN Node Monitor")
    row = write_line(
        stdscr,
        row,
        f"iface={args.iface}  rx={state.total_rx}  tx={state.total_tx}  rx_rate={rx_rate:.1f} fps  "
        f"poll_imu={args.poll_imu}  quit=q/Ctrl-C",
    )
    row = write_line(
        stdscr,
        row,
        f"CAN load {bar} {load_percent:6.2f}%  "
        f"bitrate={args.bitrate / 1000.0:.0f} kbps  "
        f"window={args.bus_window_s:.2f}s  "
        f"est={bit_rate / 1000.0:8.1f} kbps  "
        f"rx={rx_frames_w:5d}f/{rx_bits_w:7d}b  tx={tx_frames_w:5d}f/{tx_bits_w:7d}b",
    )
    row = write_line(stdscr, row, "")

    imu = state.imu
    qx, qy, qz, qw = imu.quat_xyzw
    pgx, pgy, pgz = imu.projected_gravity_b
    wx, wy, wz = imu.angular_velocity_rad_s

    row = write_line(stdscr, row, "[E2Box IMU]")
    row = write_line(
        stdscr,
        row,
        f"  req_count={imu.req_count}  quat_count={imu.quat_count}  gyro_count={imu.gyro_count}",
    )
    row = write_line(
        stdscr,
        row,
        f"  quat_xyzw=({qx:+.5f}, {qy:+.5f}, {qz:+.5f}, {qw:+.5f})  age={fmt_age(imu.quat_last_t, now)}",
    )
    row = write_line(
        stdscr,
        row,
        f"  projected_gravity_b=({pgx:+.5f}, {pgy:+.5f}, {pgz:+.5f})",
    )
    row = write_line(
        stdscr,
        row,
        f"  gyro_rad_s=({wx:+.5f}, {wy:+.5f}, {wz:+.5f})  age={fmt_age(imu.gyro_last_t, now)}",
    )
    row = write_line(stdscr, row, "")

    row = write_line(stdscr, row, "[SPG/MIT Actuators]")
    row = write_line(
        stdscr,
        row,
        "  CANID  Count  Age       Last              En   Temp  Iq[A]    Speed[dps]  Pos[rad]   Raw",
    )

    for can_id in args.spg_can_ids:
        motor = state.motors.get(can_id)

        if motor is None:
            row = write_line(
                stdscr,
                row,
                f"  {can_id:03X}  {0:5d}  {'never':>8}  {'-':16s}  {'-':>3}  "
                f"{'-':>4}  {'-':>7}  {'-':>10}  {'-':>8}  -",
            )
            continue

        en = "-" if motor.enabled_hint is None else ("Y" if motor.enabled_hint else "N")
        temp = "-" if motor.temp_c is None else f"{motor.temp_c:d}"
        iq = "-" if motor.iq_a_approx is None else f"{motor.iq_a_approx:+.2f}"
        speed = "-" if motor.speed_dps is None else f"{motor.speed_dps:d}"
        pos = "-" if motor.position_rad is None else f"{motor.position_rad:+.4f}"
        raw = fmt_hex_data(motor.raw_data, 8)

        row = write_line(
            stdscr,
            row,
            f"  {can_id:03X}  {motor.rx_count:5d}  {fmt_age(motor.last_t, now):>8}  "
            f"{motor.last_kind:16s}  {en:>3}  {temp:>4}  {iq:>7}  {speed:>10}  {pos:>8}  {raw}",
        )

    row = write_line(stdscr, row, "")

    row = write_line(stdscr, row, "[Recent CAN IDs]")
    recent = sorted(
        state.raw_frames.values(),
        key=lambda x: x.last_t,
        reverse=True,
    )[: max(0, args.recent_count)]

    for raw in recent:
        row = write_line(
            stdscr,
            row,
            f"  {raw.can_id:03X}  count={raw.rx_count:6d}  age={fmt_age(raw.last_t, now)}  "
            f"dlc={raw.last_dlc}  data={fmt_hex_data(raw.last_data, raw.last_dlc)}",
        )

    stdscr.refresh()


def curses_main(stdscr, args: argparse.Namespace) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    state = MonitorState()
    state.bus_load.window_s = args.bus_window_s
    state.bus_load.bitrate = args.bitrate

    sock = open_can_socket(args.iface)

    next_imu_poll_t = 0.0
    next_render_t = 0.0

    try:
        while True:
            now = monotonic()

            if args.poll_imu and now >= next_imu_poll_t:
                try:
                    payload = bytes([E2BOX_CMD_GET_ALL])
                    send_can_frame(sock, E2BOX_REQ_ID, payload)
                    tx_bits = estimate_classical_can_bits(
                        len(payload),
                        is_eff=False,
                        is_rtr=False,
                        stuff_factor=args.stuff_factor,
                    )
                    state.bus_load.add(now, tx_bits, "tx")
                    state.total_tx += 1
                except OSError:
                    pass
                next_imu_poll_t = now + max(1e-6, 1.0 / args.imu_poll_hz)

            while True:
                readable, _, _ = select.select([sock], [], [], 0.0)
                if not readable:
                    break

                try:
                    frame_bytes = sock.recv(CAN_FRAME_SIZE)
                except BlockingIOError:
                    break

                if len(frame_bytes) != CAN_FRAME_SIZE:
                    continue

                can_id, dlc, data, is_eff, is_rtr, is_err = parse_can_frame(frame_bytes)

                if is_err:
                    continue

                handle_rx_frame(args, state, can_id, dlc, data, is_eff, is_rtr)

            if now >= next_render_t:
                render_dashboard(stdscr, args, state)
                next_render_t = now + max(1e-6, 1.0 / args.ui_hz)

            ch = stdscr.getch()
            if ch in (ord("q"), ord("Q")):
                break

            time.sleep(0.001)

    finally:
        sock.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Realtime terminal monitor for MuJoCo virtual CAN nodes."
    )

    parser.add_argument(
        "--iface",
        default="can0",
        help="SocketCAN interface name. Default: vcan0",
    )

    parser.add_argument(
        "--poll-imu",
        action="store_true",
        help="Periodically send E2Box GET_ALL request 0x221#03.",
    )

    parser.add_argument(
        "--imu-poll-hz",
        type=float,
        default=400.0,
        help="IMU polling rate when --poll-imu is enabled. Default: 400 Hz",
    )

    parser.add_argument(
        "--ui-hz",
        type=float,
        default=24.0,
        help="Terminal refresh rate. Default: 24 Hz",
    )

    parser.add_argument(
        "--bitrate",
        type=float,
        default=1_000_000.0,
        help="Classical CAN bitrate in bit/s for load estimation. Default: 1000000",
    )

    parser.add_argument(
        "--bus-window-s",
        type=float,
        default=1.0,
        help="Sliding window length for CAN load estimation. Default: 1.0 s",
    )

    parser.add_argument(
        "--stuff-factor",
        type=float,
        default=1.15,
        help="Approximate bit-stuffing multiplier for CAN load estimation. Default: 1.15",
    )

    parser.add_argument(
        "--bus-bar-width",
        type=int,
        default=32,
        help="ASCII bar width for CAN load display. Default: 32",
    )

    parser.add_argument(
        "--spg-can-ids",
        type=parse_can_id_list,
        default=tuple(range(0x141, 0x14D)),
        help="Comma-separated SPG CAN IDs to display. Default: 0x141..0x14C",
    )

    parser.add_argument(
        "--tau-max-nm",
        type=float,
        default=33.0,
        help="Torque/current full-scale approximation used for status decode. Default: 33",
    )

    parser.add_argument(
        "--feedback-pos-max-rad",
        type=float,
        default=12.56,
        help="SPG MIT feedback position full-scale rad. Default: 12.56",
    )

    parser.add_argument(
        "--iq-full-scale-count",
        type=float,
        default=2048.0,
        help="Iq count full scale. Default: 2048",
    )

    parser.add_argument(
        "--iq-full-scale-current-a",
        type=float,
        default=33.0,
        help="Iq current full scale in ampere. Default: 33",
    )

    parser.add_argument(
        "--recent-count",
        type=int,
        default=12,
        help="Number of recent raw CAN IDs to display. Default: 12",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    signal.signal(signal.SIGINT, signal.default_int_handler)

    try:
        curses.wrapper(curses_main, args)
    except KeyboardInterrupt:
        return 130
    except OSError as e:
        print(f"[can_node_monitor] SocketCAN error: {e}")
        print("Check that the interface exists and is up, for example:")
        print("  sudo modprobe can can_raw vcan")
        print("  sudo ip link add dev vcan0 type vcan")
        print("  sudo ip link set up vcan0")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
