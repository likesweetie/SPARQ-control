import logging
import threading
from collections import defaultdict
from collections.abc import Callable

from .frame import CANFrame
from .can_types import CANFrameCallback


logger = logging.getLogger(__name__)


class CANDispatcher:
    """
    Dispatch CAN frames to callbacks based on CAN ID.

    Responsibilities:
    - Register callbacks by CAN ID
    - Register wildcard callbacks
    - Dispatch received frames
    - Isolate callback exceptions
    """

    def __init__(self) -> None:
        self._callbacks_by_id: dict[int, list[CANFrameCallback]] = defaultdict(list)
        self._wildcard_callbacks: list[CANFrameCallback] = []
        self._lock = threading.Lock()

    def register(self, can_id: int, callback: CANFrameCallback) -> None:
        """
        Register a callback for a specific CAN ID.
        """
        with self._lock:
            self._callbacks_by_id[can_id].append(callback)

    def unregister(self, can_id: int, callback: CANFrameCallback) -> None:
        """
        Unregister a callback from a specific CAN ID.
        """
        with self._lock:
            callbacks = self._callbacks_by_id.get(can_id)

            if callbacks is None:
                return

            try:
                callbacks.remove(callback)
            except ValueError:
                return

            if not callbacks:
                del self._callbacks_by_id[can_id]

    def register_wildcard(self, callback: CANFrameCallback) -> None:
        """
        Register a callback that receives every CAN frame.
        Useful for logging, monitoring, and debugging.
        """
        with self._lock:
            self._wildcard_callbacks.append(callback)

    def unregister_wildcard(self, callback: CANFrameCallback) -> None:
        """
        Unregister a wildcard callback.
        """
        with self._lock:
            try:
                self._wildcard_callbacks.remove(callback)
            except ValueError:
                return

    def dispatch(self, frame: CANFrame) -> int:
        """
        Dispatch a CAN frame to matching callbacks.

        Returns:
            Number of callbacks called.
        """
        with self._lock:
            callbacks = list(self._callbacks_by_id.get(frame.can_id, []))
            wildcard_callbacks = list(self._wildcard_callbacks)
 
        called_count = 0

        for callback in callbacks:
            try:
                callback(frame)
                called_count += 1
            except Exception:
                logger.exception(
                    "CAN callback failed: can_id=0x%X",
                    frame.can_id,
                )

        for callback in wildcard_callbacks:
            try:
                callback(frame)
                called_count += 1
            except Exception:
                logger.exception(
                    "CAN wildcard callback failed: can_id=0x%X",
                    frame.can_id,
                )

        return called_count

    def clear(self) -> None:
        """
        Remove all registered callbacks.
        """
        with self._lock:
            self._callbacks_by_id.clear()
            self._wildcard_callbacks.clear()

    def registered_ids(self) -> list[int]:
        """
        Return registered CAN IDs.
        """
        with self._lock:
            return list(self._callbacks_by_id.keys())
