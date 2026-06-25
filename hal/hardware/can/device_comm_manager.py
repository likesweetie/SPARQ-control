import time
import threading

from dataclasses import dataclass
from copy import deepcopy


@dataclass
class FreshnessStatus:
    is_online: bool = False
    is_stale: bool = True

    rx_count: int = 0
    timeout_count: int = 0
    decode_error_count: int = 0

    last_rx_t: float = 0.0
    last_fault_t: float = 0.0


class BestEffortCommManager:
    """
    Latest-value based communication monitor.

    This manager does not pair TX and RX.
    It only tracks whether the latest received value is fresh.
    """

    def __init__(self, timeout: float):
        self.timeout = timeout
        self._status = FreshnessStatus()
        self._lock = threading.Lock()

    def mark_rx(self, rx_time: float | None = None) -> None:
        now = rx_time if rx_time is not None else time.monotonic()

        with self._lock:
            self._status.rx_count += 1
            self._status.last_rx_t = now
            self._status.is_stale = False
            self._status.is_online = True

    def mark_decode_error(self) -> None:
        with self._lock:
            self._status.decode_error_count += 1
            self._status.last_fault_t = time.monotonic()

    def update_fault_flags(self) -> FreshnessStatus:
        now = time.monotonic()

        with self._lock:
            was_stale = self._status.is_stale

            if self._status.last_rx_t <= 0.0:
                self._status.is_stale = True
                self._status.is_online = False

                if not was_stale:
                    self._status.timeout_count += 1
                    self._status.last_fault_t = now

                return deepcopy(self._status)

            self._status.is_stale = (now - self._status.last_rx_t) > self.timeout
            self._status.is_online = not self._status.is_stale

            # Count only the transition from fresh to stale.
            if self._status.is_stale and not was_stale:
                self._status.timeout_count += 1
                self._status.last_fault_t = now

            return deepcopy(self._status)

    def get_status(self) -> FreshnessStatus:
        with self._lock:
            return deepcopy(self._status)

    def is_fresh(self) -> bool:
        status = self.update_fault_flags()
        return not status.is_stale
    

@dataclass
class PendingTransaction:
    expected_ids: set[int]
    received_ids: set[int]
    created_t: float
    timeout: float
    event: threading.Event


class TransactionManager:
    """
    Request-response transaction tracker.

    This manager tracks whether expected response CAN IDs have been received.
    It does not guarantee reliable communication by itself.
    """

    def __init__(self):
        self._pending: PendingTransaction | None = None
        self._lock = threading.Lock()

    def start_transaction(
        self,
        expected_ids: set[int],
        timeout: float,
    ) -> PendingTransaction:
        tx = PendingTransaction(
            expected_ids=set(expected_ids),
            received_ids=set(),
            created_t=time.monotonic(),
            timeout=timeout,
            event=threading.Event(),
        )

        with self._lock:
            self._pending = tx

        return tx

    def mark_rx(self, can_id: int) -> None:
        with self._lock:
            pending = self._pending

            if pending is None:
                return

            pending.received_ids.add(can_id)

            if pending.expected_ids <= pending.received_ids:
                pending.event.set()

    def wait(self, timeout: float | None = None) -> bool:
        with self._lock:
            pending = self._pending

        if pending is None:
            return False

        wait_timeout = pending.timeout if timeout is None else timeout
        return pending.event.wait(timeout=wait_timeout)

    def is_done(self) -> bool:
        with self._lock:
            if self._pending is None:
                return False

            return self._pending.expected_ids <= self._pending.received_ids

    def is_expired(self) -> bool:
        with self._lock:
            if self._pending is None:
                return False

            return (time.monotonic() - self._pending.created_t) > self._pending.timeout

    def clear(self) -> None:
        with self._lock:
            self._pending = None