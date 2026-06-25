from abc import ABC, abstractmethod

from hal.can_bus import CANFrame


class CANDeviceDriverBase(ABC):
    def __init__(
        self,
        name: str,
        protocol,
        comm_manager=None,
    ):
        self._name = name
        self.protocol = protocol
        self.comm_manager = comm_manager

    @property
    def name(self) -> str:
        return self._name

    def rx_can_ids(self) -> list[int]:
        return self.protocol.rx_can_ids()

    @abstractmethod
    def on_frame(self, frame: CANFrame) -> None:
        raise NotImplementedError