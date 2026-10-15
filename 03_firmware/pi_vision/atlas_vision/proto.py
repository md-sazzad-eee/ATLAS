"""ATLAS wire protocol (vendored copy of 04_ros2_ws/src/atlas_bridge/atlas_bridge/proto.py; the test suite
checks both copies are identical below the docstring and against the C vectors)."""
import struct

MSG_HEARTBEAT, MSG_TELEMETRY, MSG_TASK_CMD, MSG_TASK_RESULT = 0x01, 0x02, 0x10, 0x11
MSG_DRIVE_CMD, MSG_VISION_TAG, MSG_SHUTDOWN, MSG_ACK = 0x20, 0x30, 0x40, 0x7F
MSG_SET_POSE = 0x21
MSG_DOCK_CMD, MSG_BAY_STATUS, MSG_CARRIER_STATUS, MSG_DOCK_RELAY = 0x50, 0x51, 0x52, 0x53
SET_POSE = struct.Struct("<iih")
DRIVE_CMD = struct.Struct("<hhH")
HB_ESTOP, HB_CLEAR_FAULTS = 0x01, 0x80
MAX_PAYLOAD = 96

TELEMETRY = struct.Struct("<IHBBhhHHiihB")       # atlas_telemetry_t (27 bytes)
TASK_CMD = struct.Struct("<HBiiH")               # atlas_task_cmd_t
TASK_RESULT = struct.Struct("<HBffH")            # atlas_task_result_t
VISION_TAG = struct.Struct("<HhhhhB")            # atlas_vision_tag_t
HEARTBEAT = struct.Struct("<IB")
DOCK_CMD = struct.Struct("<BBB")                 # bay, cmd (0 dock 1 undock 2 reset), expected agent (0 any 1 S1 2 S2)
BAY_STATUS = struct.Struct("<BbBBbBBHHHH")       # atlas_bay_status_t (15 bytes)
CARRIER_STATUS = struct.Struct("<IHBhhHiih")     # atlas_carrier_status_t (23 bytes)
TASK_KINDS = ["inspect", "measure", "calibration_check", "recalibrate", "return_to_dock"]


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out, block = bytearray(), bytearray()
    for b in data:
        if b == 0:
            out += bytes([len(block) + 1]) + block; block = bytearray()
        else:
            block.append(b)
            if len(block) == 254:
                out += bytes([255]) + block; block = bytearray()
    out += bytes([len(block) + 1]) + block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        if code == 0:
            raise ValueError("zero in COBS data")
        i += 1
        blk = data[i:i + code - 1]
        if len(blk) != code - 1 or 0 in blk:
            raise ValueError("bad COBS block")
        out += blk; i += code - 1
        if code != 255 and i < len(data):
            out.append(0)
    return bytes(out)


def frame_build(mtype: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    raw = bytes([mtype, seq & 0xFF]) + payload
    raw += struct.pack("<H", crc16(raw))
    return cobs_encode(raw) + b"\x00"


def frame_parse(frame: bytes):
    """frame without the trailing 0x00 -> (type, seq, payload); raises ValueError on corruption."""
    raw = cobs_decode(frame)
    if len(raw) < 4 or struct.unpack("<H", raw[-2:])[0] != crc16(raw[:-2]):
        raise ValueError("CRC mismatch")
    return raw[0], raw[1], raw[2:-2]


class Receiver:
    """Byte-stream reassembler (UART); UDP datagrams may also carry several frames."""
    def __init__(self):
        self.buf, self.errors = bytearray(), 0

    def feed(self, data: bytes):
        out = []
        for b in data:
            if b == 0:
                if self.buf:
                    try:
                        out.append(frame_parse(bytes(self.buf)))
                    except ValueError:
                        self.errors += 1
                self.buf = bytearray()
            else:
                self.buf.append(b)
        return out
