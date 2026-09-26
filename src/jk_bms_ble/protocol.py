"""JK-BMS BLE protocol (JK02_24S / JK02_32S): commands, frame assembly, decoding.

Pure Python, no ROS or bleak dependency, so it can be unit tested anywhere.

Frame layout and offsets follow the two reference implementations:

* patman15/BMS_BLE-HA -> aiobmsble ``aiobmsble/bms/jikong_bms.py``
* syssi/esphome-jk-bms ``components/jk_bms_ble/jk_bms_ble.cpp``

The JK-BD4A8S4P (hw 11.x, sw 11.x) speaks JK02_32S. The old RS485/UART
protocol with the ``0x4E 0x57`` header is NOT used over BLE.

Offsets below are for JK02_32S. JK02_24S (firmware < 11) uses the same layout
with 8 fewer cell slots, i.e. cell-related fields shifted by -16 bytes and all
later fields by -32 bytes.
"""

import enum
import struct
from collections import namedtuple
from dataclasses import dataclass, field
from typing import List, Optional

# GATT: one characteristic is used for write and notify on most modules.
# Some modules expose two characteristics with the same UUID (one write, one
# notify), so callers must select them by property, not by UUID alone.
SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

HEADER_RESPONSE = b"\x55\xaa\xeb\x90"
HEADER_COMMAND = b"\xaa\x55\x90\xeb"  # byte order swapped compared to responses
BT_MODULE_MSG = b"AT\r\n"  # sporadically emitted by the BLE module itself

FRAME_LENGTH = 300  # checksum is always byte 299, even if the BMS sends more
COMMAND_LENGTH = 20
MAX_BUFFER = 2 * FRAME_LENGTH

FRAME_TYPE_SETTINGS = 0x01
FRAME_TYPE_CELL_INFO = 0x02
FRAME_TYPE_DEVICE_INFO = 0x03

CMD_CELL_INFO = 0x96  # also starts continuous cell info streaming
CMD_DEVICE_INFO = 0x97

TEMPERATURE_NOT_AVAILABLE = -2000  # raw value (x0.1 degC) of an unplugged NTC ("NA")

# Error bitmask names (JK02), index = bit number. From esphome-jk-bms.
ERROR_NAMES = (
    "Wire resistance",                      # bit 0
    "MOSFET overtemperature",               # bit 1
    "Cell count is not equal to settings",  # bit 2
    "",                                     # bit 3
    "Battery is fully charged",             # bit 4
    "Battery pack overvoltage",             # bit 5
    "Charge overcurrent",                   # bit 6
    "Charge short circuit",                 # bit 7
    "Charge overtemperature",               # bit 8
    "Charge undertemperature",              # bit 9
    "Coprocessor communication error",      # bit 10
    "Cell undervoltage",                    # bit 11
    "Battery pack undervoltage",            # bit 12
    "Discharge overcurrent",                # bit 13
    "Discharge short circuit",              # bit 14
    "Discharge overtemperature",            # bit 15
    "Charging MOSFET abnormal",             # bit 16
    "Discharging MOSFET abnormal",          # bit 17
    "GPS disconnected",                     # bit 18
    "Modify password in time",              # bit 19
    "Discharge on failed",                  # bit 20
    "Battery overtemperature",              # bit 21
    "Temperature sensor anomaly",           # bit 22
    "PL module anomaly",                    # bit 23
    "SCP release failed",                   # bit 24
    "Discharge OCP II",                     # bit 25
    "Discharge OCP III",                    # bit 26
    "Discharge undertemperature alarm",     # bit 27
    "GPS remote lock",                      # bit 28
)


class ProtocolVersion(enum.Enum):
    JK02_24S = "JK02_24S"
    JK02_32S = "JK02_32S"

    @property
    def offset(self) -> int:
        return -32 if self is ProtocolVersion.JK02_24S else 0

    @property
    def max_cells(self) -> int:
        return 24 if self is ProtocolVersion.JK02_24S else 32

    @staticmethod
    def from_sw_version(sw_version: str) -> "ProtocolVersion":
        """Firmware >= 11 uses JK02_32S (same rule as aiobmsble)."""
        return ProtocolVersion.JK02_32S if sw_major(sw_version) >= 11 else ProtocolVersion.JK02_24S


class ProtocolError(ValueError):
    """Raised when a frame cannot be decoded."""


def checksum(data: bytes) -> int:
    """Byte sum modulo 256 (used for commands and responses)."""
    return sum(data) & 0xFF


def build_command(cmd: int, value: bytes = b"") -> bytes:
    """Build a 20 byte command frame: header, cmd, len, 13 byte value, checksum."""
    if not 0 <= cmd <= 0xFF:
        raise ValueError("command out of range: %r" % cmd)
    if len(value) > 13:
        raise ValueError("command value too long (%d > 13 bytes)" % len(value))
    frame = HEADER_COMMAND + bytes([cmd, len(value)]) + value + bytes(13 - len(value))
    return frame + bytes([checksum(frame)])


def verify_frame(frame: bytes) -> bool:
    return (
        len(frame) >= FRAME_LENGTH
        and frame.startswith(HEADER_RESPONSE)
        and checksum(frame[: FRAME_LENGTH - 1]) == frame[FRAME_LENGTH - 1]
    )


AssembledFrame = namedtuple("AssembledFrame", ["frame_type", "data", "checksum_ok"])


class FrameAssembler:
    """Reassembles ~300 byte response frames from MTU sized notify chunks."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, chunk: bytes) -> List[AssembledFrame]:
        """Add one notification payload, return all frames completed by it."""
        chunk = bytes(chunk)
        if chunk.startswith(BT_MODULE_MSG):
            chunk = chunk[len(BT_MODULE_MSG):]
        if not chunk:
            return []

        if chunk.startswith(HEADER_RESPONSE):
            self._buffer.clear()  # a new frame always restarts assembly
        self._buffer.extend(chunk)
        self._resync()

        frames = []
        while len(self._buffer) >= FRAME_LENGTH:
            data = bytes(self._buffer[:FRAME_LENGTH])
            del self._buffer[:FRAME_LENGTH]
            frames.append(AssembledFrame(data[4], data, verify_frame(data)))
            self._resync()  # drop trailers (e.g. "BMS ready" 0xC8 message)

        if len(self._buffer) > MAX_BUFFER:
            self._buffer.clear()
        return frames

    def _resync(self) -> None:
        """Discard everything before the next response header."""
        if self._buffer.startswith(HEADER_RESPONSE):
            return
        idx = self._buffer.find(HEADER_RESPONSE)
        if idx >= 0:
            del self._buffer[:idx]
        else:
            # keep a possible partial header at the very end
            keep = 0
            for n in range(len(HEADER_RESPONSE) - 1, 0, -1):
                if self._buffer.endswith(HEADER_RESPONSE[:n]):
                    keep = n
                    break
            del self._buffer[: len(self._buffer) - keep]


def _u8(d: bytes, pos: int) -> int:
    return d[pos]


def _u16(d: bytes, pos: int) -> int:
    return struct.unpack_from("<H", d, pos)[0]


def _i16(d: bytes, pos: int) -> int:
    return struct.unpack_from("<h", d, pos)[0]


def _u32(d: bytes, pos: int) -> int:
    return struct.unpack_from("<I", d, pos)[0]


def _i32(d: bytes, pos: int) -> int:
    return struct.unpack_from("<i", d, pos)[0]


def _str(d: bytes, start: int, end: int) -> str:
    return d[start:end].split(b"\x00", 1)[0].decode("ascii", errors="replace").strip()


def sw_major(sw_version: str) -> int:
    digits = ""
    for ch in sw_version.strip():
        if not ch.isdigit():
            break
        digits += ch
    return int(digits) if digits else 0


def error_names(bitmask: int) -> List[str]:
    """Human readable names for all set error bits."""
    names = []
    for bit in range(32):
        if bitmask & (1 << bit):
            name = ERROR_NAMES[bit] if bit < len(ERROR_NAMES) else ""
            names.append(name or "Unknown error bit %d" % bit)
    return names


@dataclass
class DeviceInfo:
    model: str
    hw_version: str
    sw_version: str
    name: str
    serial_number: str

    @property
    def protocol(self) -> ProtocolVersion:
        return ProtocolVersion.from_sw_version(self.sw_version)


@dataclass
class CellInfo:
    cell_voltages: List[float]              # V, enabled cells only
    enabled_cell_count: int
    total_voltage: float                    # V
    current: float                          # A, + charging / - discharging
    state_of_charge: int                    # %
    remaining_capacity: float               # Ah
    full_charge_capacity: float             # Ah
    cycle_count: int
    state_of_health: int                    # %
    mosfet_temperature: Optional[float]     # degC
    temperatures: List[Optional[float]] = field(default_factory=list)  # [T1, T2], None = "NA"
    charge_mosfet: bool = False
    discharge_mosfet: bool = False
    balancing_current: float = 0.0          # A
    error_bitmask: int = 0
    frame_counter: int = 0

    @property
    def errors(self) -> List[str]:
        return error_names(self.error_bitmask)


def parse_device_info(frame: bytes) -> DeviceInfo:
    if len(frame) < FRAME_LENGTH or frame[4] != FRAME_TYPE_DEVICE_INFO:
        raise ProtocolError("not a device info frame")
    return DeviceInfo(
        model=_str(frame, 6, 22),
        hw_version=_str(frame, 22, 30),
        sw_version=_str(frame, 30, 38),
        name=_str(frame, 46, 62),
        serial_number=_str(frame, 86, 94),
    )


def _temperature(frame: bytes, pos: int, present_mask: int, bit: int) -> Optional[float]:
    raw = _i16(frame, pos)
    if not present_mask & (1 << bit) or raw == TEMPERATURE_NOT_AVAILABLE:
        return None
    return raw / 10.0


def parse_cell_info(frame: bytes, protocol: ProtocolVersion = ProtocolVersion.JK02_32S) -> CellInfo:
    """Decode a cell info (type 0x02) frame. The checksum must be verified before."""
    if len(frame) < FRAME_LENGTH or not frame.startswith(HEADER_RESPONSE):
        raise ProtocolError("frame too short or bad header")
    if frame[4] != FRAME_TYPE_CELL_INFO:
        raise ProtocolError("not a cell info frame (type 0x%02X)" % frame[4])

    o = protocol.offset       # for fields after the cell resistance block
    oc = o // 2               # for fields in the cell block

    enabled_mask = _u32(frame, 70 + oc)
    cell_voltages = [
        _u16(frame, 6 + 2 * i) / 1000.0
        for i in range(protocol.max_cells)
        if enabled_mask & (1 << i)
    ]

    # Temperature sensor present mask; bit order differs between versions.
    temp_mask = _i16(frame, 214 + o)
    if protocol is ProtocolVersion.JK02_32S:
        mosfet = _temperature(frame, 144, temp_mask, 0)
        t1 = _temperature(frame, 162, temp_mask, 1)
        t2 = _temperature(frame, 164, temp_mask, 2)
        errors = _u32(frame, 166)
    else:
        t1 = _temperature(frame, 130, temp_mask, 0)
        t2 = _temperature(frame, 132, temp_mask, 1)
        mosfet = _temperature(frame, 134, temp_mask, 2)
        errors = _u16(frame, 136)

    return CellInfo(
        cell_voltages=cell_voltages,
        enabled_cell_count=len(cell_voltages),
        total_voltage=_u32(frame, 150 + o) / 1000.0,
        current=_i32(frame, 158 + o) / 1000.0,
        balancing_current=_i16(frame, 170 + o) / 1000.0,
        state_of_charge=_u8(frame, 173 + o),
        remaining_capacity=_u32(frame, 174 + o) / 1000.0,
        full_charge_capacity=_u32(frame, 178 + o) / 1000.0,
        cycle_count=_u32(frame, 182 + o),
        state_of_health=_u8(frame, 190 + o),
        charge_mosfet=bool(_u8(frame, 198 + o)),
        discharge_mosfet=bool(_u8(frame, 199 + o)),
        mosfet_temperature=mosfet,
        temperatures=[t1, t2],
        error_bitmask=errors,
        frame_counter=frame[5],
    )
