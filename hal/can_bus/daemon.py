import threading
import queue
import logging

from abc import ABC, abstractmethod
from dataclasses import dataclass

from .frame import CANFrame
from .bus import CANBus, SocketCANBus
from .dispatcher import CANDispatcher
from .can_types import CANFrameCallback

logger = logging.getLogger(__name__)

class CANDaemon:
    """
    A daemon for handling CAN bus communication.

    Responsibilities:
    - Receive CAN frames from CANBus
    - Dispatch received frames to registered callbacks
    - Send queued CAN frames through CANBus
    """
    def __init__(self, 
                 can_bus: CANBus,
                 dispatcher: CANDispatcher | None = None,
                 rx_timeout: float = 0.002, #[s]
                 tx_timeout: float = 0.002, #[s]
                 join_timeout: float = 1.0, #[s]
                 max_tx_queue_size: int = 4096,
                 ):
        self.can_bus: CANBus = can_bus
        self.dispatcher: CANDispatcher = dispatcher or CANDispatcher()

        self.rx_timeout: float = rx_timeout
        self.tx_timeout: float = tx_timeout
        self.join_timeout: float = join_timeout
        self.max_tx_queue_size: int = max_tx_queue_size

        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()
        self._started = False

        self._rx_thread: threading.Thread | None = None   
        self._tx_thread: threading.Thread | None = None

        self._tx_queue: queue.Queue[CANFrame] = queue.Queue(maxsize=self.max_tx_queue_size)

    def register_callback(self, can_id: int, callback: CANFrameCallback) -> None:
        self.dispatcher.register(can_id, callback)

    def unregister_callback(self, can_id: int, callback: CANFrameCallback) -> None:
        self.dispatcher.unregister(can_id, callback)

    def register_wildcard_callback(self, callback: CANFrameCallback) -> None:
        self.dispatcher.register_wildcard(callback)

    def start(self):
        with self._state_lock:
            if self._started:
                return
            
            self._stop_event.clear()
            
            can_interface_name = getattr(self.can_bus, "interface_name", "unknown")

            self._rx_thread = threading.Thread(target=self._rx_loop, name=f"CAN_RX_{can_interface_name}_THREAD")
            self._tx_thread = threading.Thread(target=self._tx_loop, name=f"CAN_TX_{can_interface_name}_THREAD")

            self._rx_thread.start()
            self._tx_thread.start()

            self._started = True

    def stop(self, join_timeout: float = None) -> None:
        with self._state_lock:
            if not self._started:
                return
            
            if join_timeout is None:
                join_timeout = self.join_timeout

            self._stop_event.set()

        if self._rx_thread is not None:
            self._rx_thread.join(timeout=join_timeout)

        if self._tx_thread is not None:
            self._tx_thread.join(timeout=join_timeout)

        with self._state_lock:
            self._started = False
            self._rx_thread = None
            self._tx_thread = None

    def send(self, frame: CANFrame, block: bool = False, timeout: float | None = None) -> bool:
        try:
            self._tx_queue.put(frame, block=block, timeout=timeout)
            return True
        except queue.Full:
            logger.warning("CAN TX queue is full. Dropped frame: can_id=0x%X", frame.can_id)
            return False

    def _rx_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self.can_bus.recv_frame(timeout=self.rx_timeout)
            except Exception:
                logger.exception("CAN RX failed")
                continue

            if frame is None:
                continue

            try:
                self.dispatcher.dispatch(frame)
            except Exception:
                logger.exception("CAN dispatch failed: can_id=0x%X", frame.can_id)

    def _tx_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                frame = self._tx_queue.get(timeout=self.tx_timeout)
            except queue.Empty:
                continue

            try:
                self.can_bus.send_frame(frame)
            except Exception:
                logger.exception("CAN TX failed: can_id=0x%X", frame.can_id)
            finally:
                self._tx_queue.task_done()
