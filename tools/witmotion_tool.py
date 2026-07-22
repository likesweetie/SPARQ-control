#!/usr/bin/env python3
"""
Read-only real-time monitor for the WitMotion BWT901CL IMU.

The BWT901CL model profile is used to decode configuration registers such as
return content/rate, filter bandwidth, installation direction, 6/9-axis
algorithm, LED state, calibration offsets, and gyroscope auto-calibration.

Supported receive protocols
---------------------------
1. WitMotion Bluetooth 20-byte protocol
   * Live data:       55 61 + 9 little-endian int16 values
                      acceleration XYZ, gyro XYZ, Euler XYZ
   * Register reply:  55 71 + start-register uint16 + 8 uint16 registers

2. Standard WIT 11-byte UART protocol
   * Streaming:       55 <type> <8-byte payload> <checksum>
   * Register reply:  55 5F <4 little-endian uint16> <checksum>

Read command used for both families
-----------------------------------
    FF AA 27 <register low> <register high>

The program is deliberately read-only. It does not unlock, modify, calibrate,
save, reset, or restore the IMU.

Typical usage
-------------
    python3 -m pip install pyserial
    python3 witmotion_tool.py --port /dev/ttyUSB0 --baud 115200
    python3 witmotion_tool.py --port /dev/ttyUSB0 --baud auto
    python3 witmotion_tool.py --list-ports
"""

from __future__ import annotations

TOOL_VERSION = "bwt901cl-v2.1-20260712"

import argparse
import csv
import curses
import math
import queue
import signal
import struct
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple

try:
    import serial
    from serial import SerialException
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial is required. Install it with: python3 -m pip install pyserial"
    ) from exc


STANDARD_PACKET_SIZE = 11
BLE_PACKET_SIZE = 20
STANDARD_TYPES = set(range(0x50, 0x5B)) | {0x5F}
BLE_TYPES = {0x61, 0x71}
ALL_TYPES = STANDARD_TYPES | BLE_TYPES
COMMON_BAUDS = (115200, 9600, 19200, 38400, 57600, 230400, 460800, 921600)

PROTOCOL_STANDARD = "standard-11B"
PROTOCOL_BLE = "bluetooth-20B"

FRAME_NAMES = {
    0x50: "TIME",
    0x51: "ACC",
    0x52: "GYRO",
    0x53: "ANGLE",
    0x54: "MAG",
    0x55: "PORT",
    0x56: "PRESS",
    0x57: "LONLAT",
    0x58: "GPS",
    0x59: "QUAT",
    0x5A: "GPS_ACC",
    0x5F: "STD_REG_REPLY",
    0x61: "BLE_ACC_GYRO_ANGLE",
    0x71: "BLE_REG_REPLY",
}

# Standard WIT protocol output mask at register 0x02. This is not the BLE map.
OUTPUT_BITS = (
    (0x0001, "time"),
    (0x0002, "acc"),
    (0x0004, "gyro"),
    (0x0008, "angle"),
    (0x0010, "mag"),
    (0x0020, "port"),
    (0x0040, "pressure"),
    (0x0080, "gps-position"),
    (0x0100, "gps-speed"),
    (0x0200, "quaternion"),
    (0x0400, "gps-accuracy"),
)

# BWT901CL Bluetooth 2.0 register codes from the model datasheet.
BWT901CL_RATE_CODES = {
    0x01: "0.2 Hz",
    0x02: "0.5 Hz",
    0x03: "1 Hz",
    0x04: "2 Hz",
    0x05: "5 Hz",
    0x06: "10 Hz (factory default)",
    0x07: "20 Hz",
    0x08: "50 Hz",
    0x09: "100 Hz",
    0x0A: "125 Hz",
    0x0B: "200 Hz",
    0x0C: "single return",
    0x0D: "automatic output disabled",
}

# The BWT901CL product baud rate is fixed at 115200 bps.  The register is
# retained for compatibility with the embedded WIT register map.
BWT901CL_BAUD_CODES = {
    0x00: "2400 bps",
    0x01: "4800 bps",
    0x02: "9600 bps",
    0x03: "19200 bps",
    0x04: "38400 bps",
    0x05: "57600 bps",
    0x06: "115200 bps",
    0x07: "230400 bps",
    0x08: "460800 bps",
    0x09: "921600 bps",
}

BWT901CL_BANDWIDTH_CODES = {
    0x00: "256 Hz",
    0x01: "188 Hz",
    0x02: "98 Hz",
    0x03: "42 Hz",
    0x04: "20 Hz (factory default)",
    0x05: "10 Hz",
    0x06: "5 Hz",
}

BWT901CL_IMPORTANT_SETTINGS = (
    0x02,  # return content mask
    0x03,  # return rate
    0x04,  # baud register (product fixed at 115200)
    0x1B,  # LED
    0x1F,  # bandwidth
    0x20,  # gyro range
    0x21,  # acceleration range
    0x22,  # sleep state
    0x23,  # installation direction
    0x24,  # 6/9-axis algorithm
    0x25,  # dynamic filter coefficient
    0x63,  # gyro automatic calibration switch on BWT901CL firmware
    # Calibration offsets are placed after the main settings so the important
    # operating configuration remains visible on shorter terminals.
    0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A,
    0x0B, 0x0C, 0x0D,
)


@dataclass(frozen=True)
class RegisterReply:
    protocol: str
    start_address: Optional[int]
    values: Tuple[int, ...]


@dataclass
class MonitorState:
    lock: threading.RLock = field(default_factory=threading.RLock)
    values: Dict[str, object] = field(default_factory=dict)
    frame_counts: Counter = field(default_factory=Counter)
    protocol_counts: Counter = field(default_factory=Counter)
    rate_hz: Dict[int, float] = field(default_factory=dict)
    rate_window_counts: Counter = field(default_factory=Counter)
    rate_window_started: float = field(default_factory=time.monotonic)
    valid_packets: int = 0
    checksum_errors: int = 0
    discarded_bytes: int = 0
    serial_errors: int = 0
    last_packet_monotonic: float = 0.0
    last_packet_wall: float = 0.0
    recent_frames: Deque[Tuple[float, str, bytes]] = field(
        default_factory=lambda: deque(maxlen=8)
    )
    registers: Dict[int, int] = field(default_factory=dict)
    settings_error: str = "not queried"
    settings_updated_wall: float = 0.0
    connected: bool = False

    def update_rate_estimates(self, now: float) -> None:
        with self.lock:
            elapsed = now - self.rate_window_started
            if elapsed < 1.0:
                return
            for packet_type in ALL_TYPES:
                current = self.rate_window_counts.get(packet_type, 0) / elapsed
                previous = self.rate_hz.get(packet_type, current)
                self.rate_hz[packet_type] = 0.55 * previous + 0.45 * current
            self.rate_window_counts.clear()
            self.rate_window_started = now


class CsvPacketLogger:
    def __init__(self, path: Optional[str]) -> None:
        self._file = None
        self._writer = None
        self._lock = threading.Lock()
        if path:
            output = Path(path).expanduser()
            output.parent.mkdir(parents=True, exist_ok=True)
            new_file = not output.exists() or output.stat().st_size == 0
            self._file = output.open("a", newline="", encoding="utf-8")
            self._writer = csv.writer(self._file)
            if new_file:
                self._writer.writerow(
                    [
                        "unix_time",
                        "monotonic_time",
                        "protocol",
                        "frame_type",
                        "frame_name",
                        "raw_hex",
                    ]
                )
                self._file.flush()

    def write(self, protocol: str, packet_type: int, packet: bytes) -> None:
        if self._writer is None or self._file is None:
            return
        with self._lock:
            self._writer.writerow(
                [
                    f"{time.time():.6f}",
                    f"{time.monotonic():.6f}",
                    protocol,
                    f"0x{packet_type:02X}",
                    FRAME_NAMES.get(packet_type, "UNKNOWN"),
                    packet.hex(" ").upper(),
                ]
            )
            self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None


class WitPacketParser:
    def __init__(
        self,
        state: MonitorState,
        register_reply_queue: "queue.Queue[RegisterReply]",
        logger: CsvPacketLogger,
        protocol_mode: str = "auto",
    ) -> None:
        if protocol_mode not in {"auto", "ble", "standard"}:
            raise ValueError(f"invalid protocol mode: {protocol_mode}")
        self.state = state
        self.register_reply_queue = register_reply_queue
        self.logger = logger
        self.protocol_mode = protocol_mode
        self.buffer = bytearray()

    def _accept_type(self, packet_type: int) -> bool:
        if self.protocol_mode == "ble":
            return packet_type in BLE_TYPES
        if self.protocol_mode == "standard":
            return packet_type in STANDARD_TYPES
        return packet_type in ALL_TYPES

    def feed(self, data: bytes) -> int:
        if not data:
            return 0
        self.buffer.extend(data)
        parsed = 0

        while True:
            header_index = self.buffer.find(b"\x55")
            if header_index < 0:
                with self.state.lock:
                    self.state.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if header_index > 0:
                with self.state.lock:
                    self.state.discarded_bytes += header_index
                del self.buffer[:header_index]

            if len(self.buffer) < 2:
                break

            packet_type = self.buffer[1]
            if not self._accept_type(packet_type):
                with self.state.lock:
                    self.state.discarded_bytes += 1
                del self.buffer[0]
                continue

            if packet_type in BLE_TYPES:
                if len(self.buffer) < BLE_PACKET_SIZE:
                    break
                packet = bytes(self.buffer[:BLE_PACKET_SIZE])
                del self.buffer[:BLE_PACKET_SIZE]
                self._handle_ble_packet(packet)
                parsed += 1
                continue

            if len(self.buffer) < STANDARD_PACKET_SIZE:
                break
            packet = bytes(self.buffer[:STANDARD_PACKET_SIZE])
            expected = sum(packet[:10]) & 0xFF
            if expected != packet[10]:
                with self.state.lock:
                    self.state.checksum_errors += 1
                    self.state.discarded_bytes += 1
                # Drop one byte, not the entire candidate packet, so the parser can resync.
                del self.buffer[0]
                continue

            del self.buffer[:STANDARD_PACKET_SIZE]
            self._handle_standard_packet(packet)
            parsed += 1

        return parsed

    @staticmethod
    def _signed_u16(value: int) -> int:
        return value - 0x10000 if value & 0x8000 else value

    def _record_packet(self, protocol: str, packet_type: int, packet: bytes) -> None:
        now_mono = time.monotonic()
        now_wall = time.time()
        with self.state.lock:
            self.state.valid_packets += 1
            self.state.frame_counts[packet_type] += 1
            self.state.protocol_counts[protocol] += 1
            self.state.rate_window_counts[packet_type] += 1
            self.state.last_packet_monotonic = now_mono
            self.state.last_packet_wall = now_wall
            self.state.recent_frames.append((now_wall, protocol, packet))
        self.logger.write(protocol, packet_type, packet)

    def _queue_reply(self, reply: RegisterReply) -> None:
        try:
            self.register_reply_queue.put_nowait(reply)
        except queue.Full:
            try:
                self.register_reply_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.register_reply_queue.put_nowait(reply)
            except queue.Full:
                pass

    def _handle_ble_packet(self, packet: bytes) -> None:
        packet_type = packet[1]
        self._record_packet(PROTOCOL_BLE, packet_type, packet)

        if packet_type == 0x61:
            ax, ay, az, gx, gy, gz, roll, pitch, yaw = struct.unpack_from(
                "<9h", packet, 2
            )
            with self.state.lock:
                self.state.values["acc_g"] = tuple(
                    value / 32768.0 * 16.0 for value in (ax, ay, az)
                )
                self.state.values["gyro_dps"] = tuple(
                    value / 32768.0 * 2000.0 for value in (gx, gy, gz)
                )
                self.state.values["euler_deg"] = tuple(
                    value / 32768.0 * 180.0 for value in (roll, pitch, yaw)
                )
            return

        # 0x71: start address (2 bytes) + eight 16-bit register values.
        start = struct.unpack_from("<H", packet, 2)[0]
        values = struct.unpack_from("<8H", packet, 4)
        with self.state.lock:
            for offset, value in enumerate(values):
                self.state.registers[(start + offset) & 0xFFFF] = value
            self._decode_ble_register_window_locked(start, values)
        self._queue_reply(RegisterReply(PROTOCOL_BLE, start, tuple(values)))

    def _decode_ble_register_window_locked(
        self, start: int, values: Tuple[int, ...]
    ) -> None:
        by_addr = {(start + index) & 0xFFFF: value for index, value in enumerate(values)}

        if all(addr in by_addr for addr in (0x3A, 0x3B, 0x3C)):
            self.state.values["mag_raw"] = tuple(
                self._signed_u16(by_addr[addr]) for addr in (0x3A, 0x3B, 0x3C)
            )

        if all(addr in by_addr for addr in (0x3D, 0x3E, 0x3F)):
            self.state.values["euler_deg"] = tuple(
                self._signed_u16(by_addr[addr]) / 32768.0 * 180.0
                for addr in (0x3D, 0x3E, 0x3F)
            )

        if 0x40 in by_addr:
            self.state.values["temperature_c"] = self._signed_u16(by_addr[0x40]) / 100.0

        if all(addr in by_addr for addr in (0x51, 0x52, 0x53, 0x54)):
            quat = tuple(
                self._signed_u16(by_addr[addr]) / 32768.0
                for addr in (0x51, 0x52, 0x53, 0x54)
            )
            self.state.values["quat_wxyz"] = quat
            self.state.values["quat_norm"] = math.sqrt(sum(v * v for v in quat))

        if 0x64 in by_addr:
            raw = by_addr[0x64]
            # Bluetooth manuals use raw=370 to mean approximately 3.70 V.
            self.state.values["battery_raw"] = raw
            if 200 <= raw <= 600:
                self.state.values["battery_voltage_v"] = raw / 100.0

    def _handle_standard_packet(self, packet: bytes) -> None:
        packet_type = packet[1]
        payload = packet[2:10]
        self._record_packet(PROTOCOL_STANDARD, packet_type, packet)

        with self.state.lock:
            if packet_type == 0x50:
                year_month, day_hour, minute_second, millis = struct.unpack("<HHHH", payload)
                year = 2000 + (year_month & 0xFF)
                month = (year_month >> 8) & 0xFF
                day = day_hour & 0xFF
                hour = (day_hour >> 8) & 0xFF
                minute = minute_second & 0xFF
                second = (minute_second >> 8) & 0xFF
                self.state.values["chip_time"] = (
                    f"{year:04d}-{month:02d}-{day:02d} "
                    f"{hour:02d}:{minute:02d}:{second:02d}.{millis:03d}"
                )
            elif packet_type == 0x51:
                ax, ay, az, temp = struct.unpack("<hhhh", payload)
                self.state.values["acc_g"] = tuple(
                    value / 32768.0 * 16.0 for value in (ax, ay, az)
                )
                self.state.values["temperature_c"] = temp / 100.0
            elif packet_type == 0x52:
                gx, gy, gz, aux = struct.unpack("<hhhh", payload)
                self.state.values["gyro_dps"] = tuple(
                    value / 32768.0 * 2000.0 for value in (gx, gy, gz)
                )
                battery_raw = aux & 0xFFFF
                self.state.values["gyro_aux_raw"] = battery_raw
                self.state.values["battery_raw"] = battery_raw
                if 200 <= battery_raw <= 600:
                    self.state.values["battery_voltage_v"] = battery_raw / 100.0
            elif packet_type == 0x53:
                roll, pitch, yaw, version = struct.unpack("<hhhh", payload)
                self.state.values["euler_deg"] = tuple(
                    value / 32768.0 * 180.0 for value in (roll, pitch, yaw)
                )
                self.state.values["version_raw"] = version & 0xFFFF
            elif packet_type == 0x54:
                mx, my, mz, aux = struct.unpack("<hhhh", payload)
                self.state.values["mag_raw"] = (mx, my, mz)
                self.state.values["mag_aux_raw"] = aux
            elif packet_type == 0x56:
                pressure = struct.unpack_from("<I", payload, 0)[0]
                altitude = struct.unpack_from("<i", payload, 4)[0]
                self.state.values["pressure_raw"] = pressure
                self.state.values["altitude_raw"] = altitude
            elif packet_type == 0x57:
                self.state.values["longitude_raw"] = struct.unpack_from("<I", payload, 0)[0]
                self.state.values["latitude_raw"] = struct.unpack_from("<I", payload, 4)[0]
            elif packet_type == 0x58:
                height, heading = struct.unpack_from("<hh", payload, 0)
                speed = struct.unpack_from("<I", payload, 4)[0]
                self.state.values["gps"] = (
                    height / 10.0,
                    heading / 100.0,
                    speed / 1000.0,
                )
            elif packet_type == 0x59:
                q0, q1, q2, q3 = struct.unpack("<hhhh", payload)
                quat = tuple(value / 32768.0 for value in (q0, q1, q2, q3))
                self.state.values["quat_wxyz"] = quat
                self.state.values["quat_norm"] = math.sqrt(sum(v * v for v in quat))

        if packet_type == 0x5F:
            values = struct.unpack("<4H", payload)
            self._queue_reply(RegisterReply(PROTOCOL_STANDARD, None, tuple(values)))


class WitSerialDevice:
    def __init__(
        self,
        port: str,
        baud: int,
        state: MonitorState,
        logger: CsvPacketLogger,
        protocol_mode: str,
        register_timeout_s: float,
    ) -> None:
        self.port = port
        self.baud = baud
        self.state = state
        self.logger = logger
        self.register_timeout_s = max(0.05, register_timeout_s)
        self.reply_queue: "queue.Queue[RegisterReply]" = queue.Queue(maxsize=64)
        self.parser = WitPacketParser(state, self.reply_queue, logger, protocol_mode)
        self.serial: Optional[serial.Serial] = None
        self.stop_event = threading.Event()
        self.reader_thread: Optional[threading.Thread] = None
        self.io_lock = threading.Lock()
        self.query_lock = threading.Lock()

    def open(self) -> None:
        self.serial = serial.Serial(
            self.port,
            self.baud,
            timeout=0.05,
            write_timeout=0.5,
            inter_byte_timeout=None,
        )
        self.serial.reset_input_buffer()
        with self.state.lock:
            self.state.connected = True
        self.reader_thread = threading.Thread(
            target=self._reader_loop,
            name="witmotion-reader",
            daemon=True,
        )
        self.reader_thread.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.reader_thread is not None:
            self.reader_thread.join(timeout=1.0)
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        with self.state.lock:
            self.state.connected = False

    def _reader_loop(self) -> None:
        assert self.serial is not None
        while not self.stop_event.is_set():
            try:
                waiting = self.serial.in_waiting
                data = self.serial.read(waiting if waiting > 0 else 1)
                self.parser.feed(data)
                self.state.update_rate_estimates(time.monotonic())
            except (SerialException, OSError):
                with self.state.lock:
                    self.state.serial_errors += 1
                    self.state.connected = False
                self.stop_event.wait(0.1)
            except Exception:
                with self.state.lock:
                    self.state.serial_errors += 1
                self.stop_event.wait(0.05)

    def _write(self, data: bytes) -> None:
        if self.serial is None or not self.serial.is_open:
            raise RuntimeError("serial port is not open")
        with self.io_lock:
            written = self.serial.write(data)
            self.serial.flush()
        if written != len(data):
            raise IOError(f"short serial write: {written}/{len(data)} bytes")

    def _clear_reply_queue(self) -> None:
        while True:
            try:
                self.reply_queue.get_nowait()
            except queue.Empty:
                return

    def _read_register_block(self, start: int) -> RegisterReply:
        self._clear_reply_queue()
        command = bytes([0xFF, 0xAA, 0x27, start & 0xFF, (start >> 8) & 0xFF])
        self._write(command)

        deadline = time.monotonic() + self.register_timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"no register reply for 0x{start:04X} within "
                    f"{self.register_timeout_s:.2f} s"
                )
            try:
                reply = self.reply_queue.get(timeout=remaining)
            except queue.Empty as exc:
                raise TimeoutError(
                    f"no register reply for 0x{start:04X} within "
                    f"{self.register_timeout_s:.2f} s"
                ) from exc

            # BLE replies identify their start address. Ignore an unrelated 0x71 window.
            if reply.start_address is not None and reply.start_address != start:
                continue
            return reply

    def read_registers(self, start: int, count: int) -> List[int]:
        if not (0 <= start <= 0xFFFF):
            raise ValueError("start address must be between 0x0000 and 0xFFFF")
        if not (1 <= count <= 512):
            raise ValueError("count must be between 1 and 512")
        if start + count - 1 > 0xFFFF:
            raise ValueError("requested range exceeds 0xFFFF")

        results: List[int] = []
        current = start
        with self.query_lock:
            while len(results) < count:
                reply = self._read_register_block(current)
                take = min(len(reply.values), count - len(results))
                block_values = list(reply.values[:take])
                results.extend(block_values)
                with self.state.lock:
                    for offset, value in enumerate(block_values):
                        self.state.registers[current + offset] = value
                current += take
        return results


class SettingsPoller:
    def __init__(self, device: WitSerialDevice, state: MonitorState, period_s: float) -> None:
        self.device = device
        self.state = state
        self.period_s = max(0.0, period_s)
        self.stop_event = threading.Event()
        self.refresh_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name="wit-settings", daemon=True)
        self.started = False

    def start(self) -> None:
        self.thread.start()
        self.started = True

    def stop(self) -> None:
        self.stop_event.set()
        self.refresh_event.set()
        if self.started:
            self.thread.join(timeout=1.5)

    def refresh_now(self) -> None:
        self.refresh_event.set()

    def _query_known_settings(self) -> None:
        # BWT901CL core configuration.  The read command is read-only:
        # FF AA 27 <start register> 00.
        #
        # 0x02..0x0D: return mask/rate/baud and accel/gyro/mag offsets.
        self.device.read_registers(0x02, 12)

        # Remaining settings are queried as independent windows so a firmware
        # revision that omits one optional register does not hide the others.
        optional_windows = (
            (0x1B, 1, "LED state"),
            (0x1F, 7, "bandwidth/range/orientation/algorithm/filter"),
            (0x63, 1, "gyroscope automatic calibration"),
        )
        optional_errors: List[str] = []
        for start, count, label in optional_windows:
            try:
                self.device.read_registers(start, count)
            except Exception as exc:
                optional_errors.append(f"{label}: {exc}")

        if optional_errors:
            with self.state.lock:
                self.state.settings_error = "partial: " + " | ".join(optional_errors)

    def _run(self) -> None:
        first = True
        while not self.stop_event.is_set():
            if first:
                first = False
                # Let the live stream identify the protocol before the first query.
                if self.stop_event.wait(0.25):
                    break
            elif self.period_s > 0:
                self.refresh_event.wait(timeout=self.period_s)
            else:
                self.refresh_event.wait()
            self.refresh_event.clear()
            if self.stop_event.is_set():
                break
            try:
                with self.state.lock:
                    self.state.settings_error = ""
                self._query_known_settings()
                with self.state.lock:
                    self.state.settings_updated_wall = time.time()
            except Exception as exc:
                with self.state.lock:
                    self.state.settings_error = str(exc)


def primary_protocol(protocol_counts: Counter) -> str:
    ble = protocol_counts.get(PROTOCOL_BLE, 0)
    standard = protocol_counts.get(PROTOCOL_STANDARD, 0)
    if ble and standard:
        return "mixed"
    if ble:
        return PROTOCOL_BLE
    if standard:
        return PROTOCOL_STANDARD
    return "unknown"


def infer_model_hint(protocol_counts: Counter, registers: Dict[int, int]) -> str:
    protocol = primary_protocol(protocol_counts)
    observed = {
        PROTOCOL_BLE: "Bluetooth compact 20-byte stream",
        PROTOCOL_STANDARD: "standard 11-byte WIT stream",
        "mixed": "mixed WIT stream",
    }.get(protocol, "waiting for valid data")
    return f"BWT901CL profile; observed transport: {observed}"


def decode_output_mask(value: int) -> str:
    enabled = [name for bit, name in OUTPUT_BITS if value & bit]
    known_mask = sum(bit for bit, _ in OUTPUT_BITS)
    unknown = value & ~known_mask
    if unknown:
        enabled.append(f"unknown-bits=0x{unknown:04X}")
    return ", ".join(enabled) if enabled else "none"


def signed_register(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def decode_register(
    addr: int, value: int, protocol_counts: Counter
) -> Tuple[str, str]:
    del protocol_counts  # Register semantics are selected by the BWT901CL profile.
    signed = signed_register(value)

    if addr == 0x02:
        return "RSW / return content", decode_output_mask(value)
    if addr == 0x03:
        return "RATE / output rate", BWT901CL_RATE_CODES.get(
            value, f"unknown BWT901CL code 0x{value:04X}"
        )
    if addr == 0x04:
        decoded = BWT901CL_BAUD_CODES.get(value, f"raw code 0x{value:04X}")
        return "BAUD / UART rate", f"{decoded}; BWT901CL external baud is fixed at 115200 bps"
    if 0x05 <= addr <= 0x07:
        axis = "XYZ"[addr - 0x05]
        return f"A{axis}OFFSET / accel bias", f"{signed / 10000.0:+.5f} g (raw={signed})"
    if 0x08 <= addr <= 0x0A:
        axis = "XYZ"[addr - 0x08]
        return f"G{axis}OFFSET / gyro bias", f"{signed / 10000.0:+.5f} deg/s (raw={signed})"
    if 0x0B <= addr <= 0x0D:
        axis = "XYZ"[addr - 0x0B]
        return f"H{axis}OFFSET / magnetic bias", f"{signed} LSB"
    if addr == 0x1B:
        return "LEDOFF / indicator LED", {0: "LED enabled", 1: "LED disabled"}.get(
            value, f"unknown value {value}"
        )
    if addr == 0x1F:
        return "BANDWIDTH / digital bandwidth", BWT901CL_BANDWIDTH_CODES.get(
            value, f"unknown code 0x{value:04X}"
        )
    if addr == 0x20:
        return "GYRORANGE", {3: "±2000 deg/s (fixed)"}.get(
            value, f"raw code 0x{value:04X}; BWT901CL specification is ±2000 deg/s"
        )
    if addr == 0x21:
        return "ACCRANGE", {
            0: "adaptive base range; product maximum ±16 g",
            3: "±16 g",
        }.get(value, f"raw code 0x{value:04X}; product maximum ±16 g")
    if addr == 0x22:
        return "SLEEP / operating state", {0: "awake / working", 1: "sleep"}.get(
            value, f"unknown value {value}"
        )
    if addr == 0x23:
        return "ORIENT / installation", {0: "horizontal", 1: "vertical"}.get(
            value, f"unknown value {value}"
        )
    if addr == 0x24:
        return "AXIS6 / fusion algorithm", {
            0: "9-axis (magnetometer-aided absolute heading)",
            1: "6-axis (gyro-integrated relative heading)",
        }.get(value, f"unknown value {value}")
    if addr == 0x25:
        return "FILTK / dynamic filter", f"K={value} (factory default 30; lower = more smoothing)"
    if addr == 0x2D:
        return "POWONSEND / output after power-on", {0: "disabled", 1: "enabled"}.get(
            value, f"unknown value {value}"
        )
    if addr == 0x2E:
        return "VERSION", f"0x{value:04X} ({value})"
    if addr == 0x63:
        # BWT901CL model datasheet defines this as an on/off switch, unlike some
        # newer WIT register maps that reuse 0x63 as a calibration-time value.
        return "GYRO / automatic gyro calibration", {
            0: "enabled",
            1: "disabled",
        }.get(value, f"firmware-specific raw value {value}")
    if addr == 0x64 and 200 <= value <= 600:
        return "battery/voltage (firmware extension)", f"{value / 100.0:.2f} V"
    return "raw BWT901CL register", str(value)


def bwt901cl_configuration_rows(
    registers: Dict[int, int], protocol_counts: Counter
) -> List[Tuple[int, str, str, Optional[int]]]:
    rows: List[Tuple[int, str, str, Optional[int]]] = []
    for addr in BWT901CL_IMPORTANT_SETTINGS:
        value = registers.get(addr)
        if value is None:
            name, _ = decode_register(addr, 0, protocol_counts)
            rows.append((addr, name, "not read", None))
        else:
            name, decoded = decode_register(addr, value, protocol_counts)
            rows.append((addr, name, decoded, value))
    return rows


def fmt_vec(value: object, digits: int = 4) -> str:
    if not isinstance(value, tuple):
        return "--"
    return "[" + ", ".join(f"{float(v):+.{digits}f}" for v in value) + "]"


def projected_gravity_from_euler(euler_deg: object) -> Optional[Tuple[float, float, float]]:
    if not isinstance(euler_deg, tuple) or len(euler_deg) != 3:
        return None
    roll = math.radians(float(euler_deg[0]))
    pitch = math.radians(float(euler_deg[1]))
    # Rz(yaw) * Ry(pitch) * Rx(roll), then R^T * [0, 0, -1].
    return (
        math.sin(pitch),
        -math.sin(roll) * math.cos(pitch),
        -math.cos(roll) * math.cos(pitch),
    )


def safe_addstr(
    stdscr: "curses._CursesWindow", y: int, x: int, text: str, attr: int = 0
) -> None:
    height, width = stdscr.getmaxyx()
    if y < 0 or y >= height or x >= width:
        return
    available = max(0, width - x - 1)
    if available <= 0:
        return
    try:
        stdscr.addnstr(y, x, text, available, attr)
    except curses.error:
        pass


def snapshot_state(state: MonitorState) -> Dict[str, object]:
    with state.lock:
        return {
            "values": dict(state.values),
            "frame_counts": Counter(state.frame_counts),
            "protocol_counts": Counter(state.protocol_counts),
            "rate_hz": dict(state.rate_hz),
            "valid_packets": state.valid_packets,
            "checksum_errors": state.checksum_errors,
            "discarded_bytes": state.discarded_bytes,
            "serial_errors": state.serial_errors,
            "last_packet_monotonic": state.last_packet_monotonic,
            "recent_frames": list(state.recent_frames),
            "registers": dict(state.registers),
            "settings_error": state.settings_error,
            "settings_updated_wall": state.settings_updated_wall,
            "connected": state.connected,
        }


def prompt_line(stdscr: "curses._CursesWindow", prompt: str) -> str:
    height, width = stdscr.getmaxyx()
    y = height - 1
    curses.echo()
    stdscr.nodelay(False)
    try:
        safe_addstr(stdscr, y, 0, " " * max(0, width - 1))
        safe_addstr(stdscr, y, 0, prompt, curses.A_BOLD)
        x = min(len(prompt), max(0, width - 2))
        stdscr.move(y, x)
        raw = stdscr.getstr(y, x, max(1, width - x - 2))
        return raw.decode("utf-8", errors="replace").strip()
    finally:
        curses.noecho()
        stdscr.nodelay(True)


def draw_dashboard(
    stdscr: "curses._CursesWindow",
    state: MonitorState,
    port: str,
    baud: int,
    protocol_mode: str,
    port_description: str,
    paused: bool,
    status_line: str,
) -> None:
    snap = snapshot_state(state)
    values: Dict[str, object] = snap["values"]  # type: ignore[assignment]
    frame_counts: Counter = snap["frame_counts"]  # type: ignore[assignment]
    protocol_counts: Counter = snap["protocol_counts"]  # type: ignore[assignment]
    rates: Dict[int, float] = snap["rate_hz"]  # type: ignore[assignment]
    registers: Dict[int, int] = snap["registers"]  # type: ignore[assignment]
    recent_frames: List[Tuple[float, str, bytes]] = snap["recent_frames"]  # type: ignore[assignment]

    stdscr.erase()
    height, _width = stdscr.getmaxyx()
    safe_addstr(stdscr, 0, 0, f"WitMotion BWT901CL — read-only live monitor [{TOOL_VERSION}]", curses.A_BOLD)
    safe_addstr(
        stdscr,
        1,
        0,
        f"port={port}  baud={baud}  parser={protocol_mode}  "
        f"connected={snap['connected']}  paused={paused}",
    )
    safe_addstr(stdscr, 2, 0, f"USB/serial: {port_description}")

    age = float("inf")
    last_packet = float(snap["last_packet_monotonic"])
    if last_packet > 0:
        age = time.monotonic() - last_packet
    stream_state = "ONLINE" if age < 1.0 else ("STALE" if age < 5.0 else "NO DATA")
    protocol = primary_protocol(protocol_counts)
    safe_addstr(
        stdscr,
        3,
        0,
        f"stream={stream_state} age={age:.3f}s observed={protocol}  "
        f"valid={snap['valid_packets']} checksum-errors={snap['checksum_errors']} "
        f"discarded={snap['discarded_bytes']} serial-errors={snap['serial_errors']}",
    )
    safe_addstr(stdscr, 4, 0, f"model inference: {infer_model_hint(protocol_counts, registers)}")

    y = 6
    safe_addstr(stdscr, y, 0, "Live decoded values", curses.A_UNDERLINE)
    y += 1
    safe_addstr(stdscr, y, 0, f"accel [g]       : {fmt_vec(values.get('acc_g'))}")
    y += 1
    safe_addstr(stdscr, y, 0, f"gyro [deg/s]    : {fmt_vec(values.get('gyro_dps'))}")
    y += 1
    safe_addstr(stdscr, y, 0, f"RPY [deg]       : {fmt_vec(values.get('euler_deg'), 3)}")
    y += 1
    gravity = projected_gravity_from_euler(values.get("euler_deg"))
    safe_addstr(stdscr, y, 0, f"projected gravity: {fmt_vec(gravity, 6)}")
    y += 1
    safe_addstr(stdscr, y, 0, f"mag [raw]       : {fmt_vec(values.get('mag_raw'), 0)}")
    y += 1
    safe_addstr(
        stdscr,
        y,
        0,
        f"quat [wxyz]     : {fmt_vec(values.get('quat_wxyz'), 5)} "
        f"norm={values.get('quat_norm', '--')}",
    )
    y += 1
    safe_addstr(
        stdscr,
        y,
        0,
        f"temperature     : {values.get('temperature_c', '--')} °C  "
        f"battery={values.get('battery_voltage_v', '--')} V",
    )
    y += 1
    safe_addstr(
        stdscr,
        y,
        0,
        "axis warning    : protocol does not identify physical X/Y orientation; check enclosure label/manual",
    )

    y += 2
    safe_addstr(stdscr, y, 0, "Observed frame rates", curses.A_UNDERLINE)
    y += 1
    rate_parts = []
    for packet_type, count in sorted(frame_counts.items()):
        rate_parts.append(
            f"{FRAME_NAMES.get(packet_type, hex(packet_type))}="
            f"{rates.get(packet_type, 0.0):.1f}Hz({count})"
        )
    safe_addstr(stdscr, y, 0, "  ".join(rate_parts) if rate_parts else "--")

    y += 2
    safe_addstr(stdscr, y, 0, "BWT901CL decoded configuration", curses.A_UNDERLINE)
    y += 1
    settings_error = str(snap["settings_error"])
    settings_updated = float(snap["settings_updated_wall"])
    if settings_error:
        safe_addstr(stdscr, y, 0, f"query status: {settings_error}")
    else:
        updated_text = time.strftime("%H:%M:%S", time.localtime(settings_updated))
        safe_addstr(stdscr, y, 0, f"query status: OK, last update {updated_text}")
    y += 1

    max_register_lines = max(0, min(len(BWT901CL_IMPORTANT_SETTINGS), height - y - 5))
    for addr, name, decoded, value in bwt901cl_configuration_rows(
        registers, protocol_counts
    )[:max_register_lines]:
        raw_text = "----" if value is None else f"0x{value:04X}"
        safe_addstr(
            stdscr,
            y,
            0,
            f"0x{addr:02X}  {raw_text:>6}  {name}: {decoded}",
        )
        y += 1

    if y < height - 5:
        y += 1
        safe_addstr(stdscr, y, 0, "Recent raw frames", curses.A_UNDERLINE)
        y += 1
        for _timestamp, frame_protocol, packet in recent_frames[-2:]:
            safe_addstr(
                stdscr,
                y,
                0,
                f"{frame_protocol} {FRAME_NAMES.get(packet[1], hex(packet[1]))}: "
                f"{packet.hex(' ').upper()}",
            )
            y += 1
            if y >= height - 3:
                break

    safe_addstr(
        stdscr,
        height - 2,
        0,
        "keys: q quit | s refresh settings | r read registers | p pause | c clear | h help",
        curses.A_REVERSE,
    )
    safe_addstr(stdscr, height - 1, 0, status_line)
    stdscr.refresh()


def show_help(stdscr: "curses._CursesWindow") -> None:
    stdscr.erase()
    lines = [
        "WitMotion monitor help",
        "",
        "q     quit",
        "s     re-read BWT901CL configuration registers",
        "r     read arbitrary register range: <start> [count]",
        "      examples: 0x02 12, 0x1F 7, 0x23 2, 0x63 1",
        "p     pause/resume display redraw; reception continues",
        "c     clear cached registers and counters",
        "h     show this page",
        "",
        "Bluetooth protocol: 55 61 live frame, 55 71 register window (20 bytes).",
        "Standard protocol: 55 5x frame with checksum (11 bytes).",
        "The tool sends only FF AA 27 register-read commands.",
        "Register decoding uses the BWT901CL Bluetooth 2.0 model profile.",
        "Press any key to return.",
    ]
    for y, line in enumerate(lines):
        safe_addstr(stdscr, y, 0, line, curses.A_BOLD if y == 0 else 0)
    stdscr.refresh()
    stdscr.nodelay(False)
    try:
        stdscr.getch()
    finally:
        stdscr.nodelay(True)


def parse_int(text: str) -> int:
    return int(text, 0)


def run_tui(
    stdscr: "curses._CursesWindow",
    device: WitSerialDevice,
    poller: SettingsPoller,
    state: MonitorState,
    protocol_mode: str,
    port_description: str,
) -> None:
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    paused = False
    status_line = ""
    last_draw = 0.0

    while not device.stop_event.is_set():
        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            break
        if key in (ord("s"), ord("S")):
            poller.refresh_now()
            status_line = "settings refresh requested"
        elif key in (ord("p"), ord("P")):
            paused = not paused
            status_line = "display paused" if paused else "display resumed"
        elif key in (ord("h"), ord("H"), ord("?")):
            show_help(stdscr)
            status_line = ""
        elif key in (ord("c"), ord("C")):
            with state.lock:
                state.registers.clear()
                state.frame_counts.clear()
                state.protocol_counts.clear()
                state.rate_hz.clear()
                state.rate_window_counts.clear()
                state.valid_packets = 0
                state.checksum_errors = 0
                state.discarded_bytes = 0
            status_line = "cached registers and counters cleared"
        elif key in (ord("r"), ord("R")):
            raw = prompt_line(stdscr, "register range <start> [count]: ")
            try:
                parts = raw.split()
                if not parts:
                    raise ValueError("no address entered")
                start = parse_int(parts[0])
                count = parse_int(parts[1]) if len(parts) > 1 else 1
                values = device.read_registers(start, count)
                status_line = " ".join(
                    f"0x{start + i:04X}=0x{value:04X}({value})"
                    for i, value in enumerate(values)
                )
            except Exception as exc:
                status_line = f"read failed: {exc}"

        now = time.monotonic()
        if not paused and now - last_draw >= 0.1:
            draw_dashboard(
                stdscr,
                state,
                device.port,
                device.baud,
                protocol_mode,
                port_description,
                paused,
                status_line,
            )
            last_draw = now
        elif paused and key != -1:
            draw_dashboard(
                stdscr,
                state,
                device.port,
                device.baud,
                protocol_mode,
                port_description,
                paused,
                status_line,
            )


def find_candidate_ports() -> List[str]:
    ports = list(list_ports.comports())
    preferred: List[str] = []
    others: List[str] = []
    for item in ports:
        dev = item.device
        description = f"{item.description} {item.manufacturer or ''}".lower()
        if any(token in dev for token in ("ttyUSB", "ttyACM", "cu.usb", "COM")) or any(
            token in description for token in ("usb", "serial", "ch340", "cp210", "ftdi")
        ):
            preferred.append(dev)
        else:
            others.append(dev)
    return preferred + others


def choose_port(requested: Optional[str]) -> str:
    if requested:
        return requested
    candidates = find_candidate_ports()
    if not candidates:
        raise RuntimeError("no serial ports found; pass --port explicitly")
    if len(candidates) > 1:
        print("Multiple serial ports found; selecting the first candidate:", file=sys.stderr)
        for index, port in enumerate(candidates):
            marker = "*" if index == 0 else " "
            print(f"  {marker} {port}", file=sys.stderr)
    return candidates[0]


def describe_port(port: str) -> str:
    for item in list_ports.comports():
        if item.device == port:
            fields = [item.description or "unknown"]
            if item.manufacturer:
                fields.append(item.manufacturer)
            if item.vid is not None and item.pid is not None:
                fields.append(f"VID:PID={item.vid:04X}:{item.pid:04X}")
            if item.serial_number:
                fields.append(f"serial={item.serial_number}")
            return " | ".join(fields)
    return "no USB descriptor available"


def count_valid_packets(raw: bytes, protocol_mode: str = "auto") -> Tuple[int, bool]:
    buffer = bytearray(raw)
    valid = 0
    found_reply = False

    def accepted(packet_type: int) -> bool:
        if protocol_mode == "ble":
            return packet_type in BLE_TYPES
        if protocol_mode == "standard":
            return packet_type in STANDARD_TYPES
        return packet_type in ALL_TYPES

    while True:
        index = buffer.find(b"\x55")
        if index < 0:
            break
        if index:
            del buffer[:index]
        if len(buffer) < 2:
            break
        packet_type = buffer[1]
        if not accepted(packet_type):
            del buffer[0]
            continue
        if packet_type in BLE_TYPES:
            if len(buffer) < BLE_PACKET_SIZE:
                break
            valid += 1
            found_reply |= packet_type == 0x71
            del buffer[:BLE_PACKET_SIZE]
            continue
        if len(buffer) < STANDARD_PACKET_SIZE:
            break
        packet = bytes(buffer[:STANDARD_PACKET_SIZE])
        if (sum(packet[:10]) & 0xFF) == packet[10]:
            valid += 1
            found_reply |= packet_type == 0x5F
            del buffer[:STANDARD_PACKET_SIZE]
        else:
            del buffer[0]
    return valid, found_reply


def probe_baud(
    port: str, baud: int, protocol_mode: str, probe_s: float = 0.45
) -> int:
    try:
        with serial.Serial(port, baud, timeout=0.04, write_timeout=0.2) as ser:
            ser.reset_input_buffer()
            collected = bytearray()
            deadline = time.monotonic() + probe_s
            while time.monotonic() < deadline:
                waiting = ser.in_waiting
                collected.extend(ser.read(waiting if waiting > 0 else 1))
            valid, reply = count_valid_packets(bytes(collected), protocol_mode)
            if valid > 0:
                return valid * 10 + (50 if reply else 0)

            ser.reset_input_buffer()
            ser.write(bytes([0xFF, 0xAA, 0x27, 0x00, 0x00]))
            ser.flush()
            deadline = time.monotonic() + 0.8
            collected.clear()
            while time.monotonic() < deadline:
                waiting = ser.in_waiting
                collected.extend(ser.read(waiting if waiting > 0 else 1))
                valid, reply = count_valid_packets(bytes(collected), protocol_mode)
                if reply:
                    return 100 + valid
            valid, reply = count_valid_packets(bytes(collected), protocol_mode)
            return valid * 10 + (50 if reply else 0)
    except (SerialException, OSError):
        return -1


def detect_baud(port: str, bauds: Sequence[int], protocol_mode: str) -> int:
    print(f"Probing baud rate on {port} ...", file=sys.stderr)
    best_baud = 0
    best_score = -1
    for baud in bauds:
        score = probe_baud(port, baud, protocol_mode)
        print(f"  {baud:>7}: score={score}", file=sys.stderr)
        if score > best_score:
            best_baud = baud
            best_score = score
    if best_score <= 0:
        raise RuntimeError(
            "could not detect a valid WIT stream/register reply; specify --baud "
            "and verify --protocol"
        )
    print(f"Selected baud: {best_baud}", file=sys.stderr)
    return best_baud


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only BWT901CL monitor supporting standard 11-byte and "
            "Bluetooth compact 20-byte receive protocols"
        )
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {TOOL_VERSION}",
    )
    parser.add_argument("--port", help="serial device, e.g. /dev/ttyUSB0")
    parser.add_argument(
        "--baud", default="115200", help="baud rate or 'auto' (default: 115200)"
    )
    parser.add_argument(
        "--protocol",
        choices=("auto", "ble", "standard"),
        default="auto",
        help="receive parser mode (default: auto)",
    )
    parser.add_argument(
        "--settings-period",
        type=float,
        default=10.0,
        help="seconds between read-only settings refreshes; 0 means manual only",
    )
    parser.add_argument(
        "--register-timeout",
        type=float,
        default=1.0,
        help="timeout for each register block reply (default: 1.0 s)",
    )
    parser.add_argument("--log", help="optional CSV path for valid raw frames")
    parser.add_argument("--list-ports", action="store_true", help="list ports and exit")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if args.list_ports:
        for item in list_ports.comports():
            print(
                f"{item.device}\t{item.description}\t{item.hwid}\t"
                f"manufacturer={item.manufacturer or '-'}"
            )
        return 0

    try:
        port = choose_port(args.port)
        if str(args.baud).lower() == "auto":
            baud = detect_baud(port, COMMON_BAUDS, args.protocol)
        else:
            baud = int(args.baud, 0)
            if baud <= 0:
                raise ValueError("baud must be positive")
        if args.register_timeout <= 0:
            raise ValueError("--register-timeout must be positive")
        if args.settings_period < 0:
            raise ValueError("--settings-period must be zero or positive")
    except Exception as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2

    state = MonitorState()
    logger = CsvPacketLogger(args.log)
    device = WitSerialDevice(
        port,
        baud,
        state,
        logger,
        args.protocol,
        args.register_timeout,
    )
    poller = SettingsPoller(device, state, args.settings_period)
    port_description = describe_port(port)

    def request_stop(_signum: int, _frame: object) -> None:
        device.stop_event.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    try:
        device.open()
        poller.start()
        if not sys.stdin.isatty() or not sys.stdout.isatty():
            print(
                "This monitor requires an interactive terminal for its curses dashboard.",
                file=sys.stderr,
            )
            return 3
        curses.wrapper(
            run_tui,
            device,
            poller,
            state,
            args.protocol,
            port_description,
        )
        return 0
    except (SerialException, OSError) as exc:
        print(f"serial error: {exc}", file=sys.stderr)
        return 1
    finally:
        poller.stop()
        device.close()
        logger.close()


if __name__ == "__main__":
    raise SystemExit(main())