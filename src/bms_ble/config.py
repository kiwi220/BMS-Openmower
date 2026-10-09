"""Parsing and validation of the ``bms_list`` parameter (no ROS imports)."""

import re
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

BASE_TYPES = ("jk", "jbd", "ant", "ant_leg", "ant_new", "auto")
PACK_CONNECTIONS = ("", "series", "parallel")
COMBINED_NAME = "combined"

_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
_MAC_RE = re.compile(r"^([0-9A-F]{2}:){5}[0-9A-F]{2}$")


@dataclass(frozen=True)
class DeviceConfig:
    name: str
    type: str
    mac: str
    cell_count: int
    nominal_capacity_ah: float
    password: str = ""
    primary: bool = False

    def bridge_dict(self) -> Dict[str, str]:
        return {"name": self.name, "type": self.type, "mac": self.mac, "password": self.password}


def _device(index: int, raw: Any) -> DeviceConfig:
    where = "bms_list[%d]" % index
    if not isinstance(raw, dict):
        raise ValueError("%s must be a mapping" % where)
    name = str(raw.get("name", "")).strip()
    if not _NAME_RE.match(name):
        raise ValueError("%s: name '%s' must match [A-Za-z][A-Za-z0-9_]* (used in topic names)" % (where, name))
    if name == COMBINED_NAME:
        raise ValueError("%s: name '%s' is reserved" % (where, name))
    kind = str(raw.get("type", "auto")).strip().lower()
    if kind not in BASE_TYPES and not kind.endswith("_bms"):
        raise ValueError(
            "%s (%s): type '%s' must be one of %s or an aiobmsble module name (*_bms)"
            % (where, name, kind, "|".join(BASE_TYPES))
        )
    mac = str(raw.get("mac", "")).strip().upper()
    if not _MAC_RE.match(mac):
        raise ValueError("%s (%s): mac '%s' is not a BLE MAC address (AA:BB:CC:DD:EE:FF)" % (where, name, mac))
    try:
        cells = int(raw.get("cell_count", 0))
        capacity = float(raw.get("nominal_capacity_ah", 0.0))
    except (TypeError, ValueError):
        raise ValueError("%s (%s): cell_count / nominal_capacity_ah must be numbers" % (where, name))
    if cells <= 0:
        raise ValueError("%s (%s): cell_count must be > 0" % (where, name))
    if capacity <= 0:
        raise ValueError("%s (%s): nominal_capacity_ah must be > 0" % (where, name))
    return DeviceConfig(
        name=name,
        type=kind,
        mac=mac,
        cell_count=cells,
        nominal_capacity_ah=capacity,
        password=str(raw.get("ble_connect_password", "") or ""),
        primary=bool(raw.get("primary", False)),
    )


def parse_bms_list(raw: Any, legacy: Optional[Dict[str, Any]] = None) -> List[DeviceConfig]:
    """Validate ``bms_list``. ``legacy`` maps the old single-BMS parameters
    (bms_mac_address, cell_count, nominal_capacity_ah, ble_connect_password)
    to a one element list if ``bms_list`` is not set."""
    if raw in (None, "", []) and legacy and legacy.get("bms_mac_address"):
        raw = [{
            "name": "main",
            "type": "jk",
            "mac": legacy["bms_mac_address"],
            "cell_count": legacy.get("cell_count", 7),
            "nominal_capacity_ah": legacy.get("nominal_capacity_ah", 10.0),
            "ble_connect_password": legacy.get("ble_connect_password", ""),
            "primary": True,
        }]
    if not isinstance(raw, list) or not raw:
        raise ValueError("parameter bms_list must be a non-empty list of BMS entries")

    devices = [_device(i, d) for i, d in enumerate(raw)]
    names = [d.name for d in devices]
    dupes = sorted({n for n in names if names.count(n) > 1})
    if dupes:
        raise ValueError("duplicate BMS names: %s" % ", ".join(dupes))
    macs = [d.mac for d in devices]
    dupes = sorted({m for m in macs if macs.count(m) > 1})
    if dupes:
        raise ValueError("duplicate BMS MAC addresses: %s" % ", ".join(dupes))
    primaries = [d.name for d in devices if d.primary]
    if len(primaries) > 1:
        raise ValueError("only one BMS may be primary, got: %s" % ", ".join(primaries))
    if not primaries and len(devices) == 1:
        devices[0] = DeviceConfig(**{**devices[0].__dict__, "primary": True})
    return devices


def parse_pack_connection(value: Any) -> str:
    value = str(value or "").strip().lower()
    if value not in PACK_CONNECTIONS:
        raise ValueError("pack_connection must be empty, 'series' or 'parallel', got '%s'" % value)
    return value


XBOT_MAX_RATE_HZ = 2.0  # OpenMower throttles its own sensors to 2 Hz
XBOT_ALL = "all"


def parse_xbot_rate(value: Any) -> float:
    try:
        rate = float(value)
    except (TypeError, ValueError):
        raise ValueError("xbot_sensors_rate_hz must be a number, got '%s'" % value)
    if not 0.0 < rate <= XBOT_MAX_RATE_HZ:
        raise ValueError("xbot_sensors_rate_hz must be > 0 and <= %.1f, got %s" % (XBOT_MAX_RATE_HZ, value))
    return rate


def parse_xbot_sensor_bms(value: Any, devices: List[DeviceConfig]) -> List[DeviceConfig]:
    """BMS published as xbot_monitoring sensors.

    "" -> the primary BMS (empty list if none is primary), "all" -> all BMS,
    otherwise the name of one entry of bms_list.
    """
    value = str(value or "").strip()
    if not value:
        return [d for d in devices if d.primary]
    if value.lower() == XBOT_ALL:
        return list(devices)
    for d in devices:
        if d.name == value:
            return [d]
    raise ValueError(
        "xbot_sensor_bms '%s' is neither 'all' nor a name from bms_list (%s)"
        % (value, ", ".join(d.name for d in devices))
    )
