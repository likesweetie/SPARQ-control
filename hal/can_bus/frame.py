from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class Frame(ABC):
    data: bytes

@dataclass
class CANFrame(Frame):
    can_id: int

    @property
    def dlc(self):
        return len(self.data)
    

#TODO will be implemented
@dataclass
class ExtendedCANFrame(Frame):
    can_id: int

    @property
    def dlc(self):
        return len(self.data)