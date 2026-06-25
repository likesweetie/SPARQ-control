from __future__ import annotations

import json
import logging
import socket
import threading
import time
from typing import Callable

from .dispatcher import CANDispatcher
from .frame import CANFrame


logger = logging.getLogger(__name__)


class CANProcessClient:
    def __init__(self, socket_path: str, connect_timeout_s: float, *, rx_enabled: bool = True) -> None:
        self.socket_path = socket_path
        self.connect_timeout_s = float(connect_timeout_s)
        self.rx_enabled = bool(rx_enabled)
        self.dispatcher = CANDispatcher()
        self._tx_sock: socket.socket | None = None
        self._rx_sock: socket.socket | None = None
        self._tx_file = None
        self._rx_file = None
        self._tx_lock = threading.Lock()
        self._rx_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def connect(self) -> None:
        deadline = time.monotonic() + self.connect_timeout_s
        while True:
            try:
                self._tx_sock, self._tx_file = self._connect_role("tx")
                if self.rx_enabled:
                    self._rx_sock, self._rx_file = self._connect_role("rx")
                    self._rx_thread = threading.Thread(
                        target=self._rx_loop,
                        name="CAN_DAEMON_CLIENT_RX",
                        daemon=True,
                    )
                    self._rx_thread.start()
                return
            except OSError:
                self.close()
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)

    def close(self) -> None:
        self._stop_event.set()
        if self._tx_file is not None:
            self._tx_file.close()
            self._tx_file = None
        if self._rx_file is not None:
            self._rx_file.close()
            self._rx_file = None
        for sock in (self._tx_sock, self._rx_sock):
            if sock is not None:
                sock.close()
        self._tx_sock = None
        self._rx_sock = None
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        self._stop_event.clear()

    def register_callback(self, can_id: int, callback: Callable[[CANFrame], None]) -> None:
        self.dispatcher.register(can_id, callback)

    def register_wildcard_callback(self, callback: Callable[[CANFrame], None]) -> None:
        self.dispatcher.register_wildcard(callback)

    def send(self, frame: CANFrame) -> bool:
        if self._tx_file is None:
            raise RuntimeError("CAN daemon client is not connected")
        request = {
            "type": "tx",
            "can_id": int(frame.can_id),
            "data": bytes(frame.data).hex(),
        }
        with self._tx_lock:
            self._write_json_line(self._tx_file, request)
            response = self._read_json_line(self._tx_file)
        if response.get("type") != "tx_result":
            raise RuntimeError(f"Unexpected CAN daemon response: {response}")
        if not bool(response.get("ok")):
            raise RuntimeError(str(response.get("error") or "CAN daemon TX failed"))
        return True

    def _connect_role(self, role: str):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(self.socket_path)
        file_obj = sock.makefile("rwb")
        self._write_json_line(file_obj, {"type": "hello", "role": role})
        response = self._read_json_line(file_obj)
        if response.get("type") != "hello_ack" or not bool(response.get("ok")):
            raise RuntimeError(f"CAN daemon rejected role {role}: {response}")
        return sock, file_obj

    def _rx_loop(self) -> None:
        file_obj = self._rx_file
        assert file_obj is not None
        while not self._stop_event.is_set():
            try:
                message = self._read_json_line(file_obj)
            except OSError:
                if not self._stop_event.is_set():
                    logger.exception("CAN daemon RX socket failed")
                return
            if message.get("type") != "rx":
                continue
            frame = CANFrame(
                can_id=int(message["can_id"]),
                data=bytes.fromhex(str(message["data"])),
            )
            self.dispatcher.dispatch(frame)

    @staticmethod
    def _write_json_line(file_obj, message: dict) -> None:
        file_obj.write(json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n")
        file_obj.flush()

    @staticmethod
    def _read_json_line(file_obj) -> dict:
        line = file_obj.readline()
        if not line:
            raise OSError("CAN daemon socket closed")
        message = json.loads(line.decode("utf-8"))
        if not isinstance(message, dict):
            raise RuntimeError("CAN daemon message must be a JSON object")
        return message
