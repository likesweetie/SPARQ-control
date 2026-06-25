from __future__ import annotations

import socket
import struct
from dataclasses import dataclass


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)


@dataclass(frozen=True)
class ParsedFrame:
    can_id: int
    dlc: int
    data: bytes
    is_eff: bool = False
    is_rtr: bool = False
    is_err: bool = False


def open_can_socket(iface: str) -> socket.socket:
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind((iface,))
    sock.setblocking(False)
    return sock


def parse_can_frame(frame_bytes: bytes) -> ParsedFrame:
    can_id_raw, dlc, data = struct.unpack(CAN_FRAME_FMT, frame_bytes)
    is_eff = bool(can_id_raw & CAN_EFF_FLAG)
    is_rtr = bool(can_id_raw & CAN_RTR_FLAG)
    is_err = bool(can_id_raw & CAN_ERR_FLAG)
    can_id = can_id_raw & (CAN_EFF_MASK if is_eff else CAN_SFF_MASK)
    dlc = int(dlc)
    if dlc < 0 or dlc > 8:
        raise ValueError(f"Invalid classical CAN DLC: {dlc}")
    return ParsedFrame(
        can_id=can_id,
        dlc=dlc,
        data=data[:dlc],
        is_eff=is_eff,
        is_rtr=is_rtr,
        is_err=is_err,
    )
