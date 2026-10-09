"""BMS values as OpenMower xbot_monitoring sensors (no ROS imports).

A sensor is a pair of topics that xbot_monitoring discovers by the regex
``/xbot_monitoring/sensors/.*/info`` and forwards to MQTT
(``sensor_infos/json``, ``sensors/<id>/data``):

    /xbot_monitoring/sensors/<id>/info  xbot_msgs/SensorInfo, latched
    /xbot_monitoring/sensors/<id>/data  xbot_msgs/SensorDataDouble | SensorDataString

Message classes and the publisher factory (rospy.Publisher) are injected, so
everything here is unit testable without ROS.
"""

import math
import re
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

from . import battery_logic as bl
from .config import DeviceConfig
from .model import BmsSample

TOPIC_PREFIX = "/xbot_monitoring/sensors/"  # absolute: xbot_monitoring matches this exact prefix
RESERVED_PREFIX = "om_"                    # OpenMower's own sensors (MowBite treats them specially)
UNSET = -1.0                               # MowBite: limits < 0 mean "not set"

DOUBLE = "double"
STRING = "string"

# Display range for cell voltages (Li-ion incl. LiFePO4). Only a gauge range,
# deliberately no critical limits: those are safety statements that belong to
# the concrete cell chemistry and BMS settings.
CELL_DISPLAY_RANGE = (2.5, 4.2)

# A loop iteration may come this fraction of the period early and still publish,
# so timing jitter of the ROS loop does not drop every other cycle.
JITTER_TOLERANCE = 0.1

Value = Union[float, str]


@dataclass(frozen=True)
class SensorDef:
    sensor_id: str
    name: str
    value_type: str             # DOUBLE | STRING
    description: str            # suffix of SensorInfo.VALUE_DESCRIPTION_*, e.g. "VOLTAGE"
    unit: str = ""
    min_value: Optional[float] = None
    max_value: Optional[float] = None
    lower_critical: Optional[float] = None
    upper_critical: Optional[float] = None


def sanitize(name: str) -> str:
    """Lower case, only [a-z0-9_], no repeated or edge underscores."""
    cleaned = re.sub(r"[^a-z0-9]+", "_", str(name).lower()).strip("_")
    if not cleaned:
        raise ValueError("BMS name '%s' gives an empty sensor id" % name)
    return cleaned


def id_prefix(device: DeviceConfig) -> str:
    return "bms_%s_" % sanitize(device.name)


def _label(device: DeviceConfig, single: bool, what: str) -> str:
    return "BMS %s" % what if single else "BMS %s %s" % (device.name, what)


def cell_ids(device: DeviceConfig) -> List[str]:
    width = max(2, len(str(device.cell_count)))
    return ["%scell_%0*d" % (id_prefix(device), width, i + 1) for i in range(device.cell_count)]


def sensor_defs(
    device: DeviceConfig, single: bool, has_mosfet_temp: bool = False, temp_count: int = 0
) -> List[SensorDef]:
    """All sensors of one BMS in a fixed order.

    Temperature sensors depend on what the BMS delivers and are therefore
    added once they appear in a sample (see temperature_layout()).
    """
    p = id_prefix(device)
    width = max(2, len(str(device.cell_count)))
    lo, hi = CELL_DISPLAY_RANGE

    def lbl(what: str) -> str:
        return _label(device, single, what)

    defs = [
        SensorDef(p + "voltage", lbl("Voltage"), DOUBLE, "VOLTAGE", "V"),
        SensorDef(p + "current", lbl("Current"), DOUBLE, "CURRENT", "A"),
        SensorDef(p + "soc", lbl("SoC"), DOUBLE, "PERCENT", "%", min_value=0.0, max_value=100.0),
    ]
    if has_mosfet_temp:
        defs.append(SensorDef(p + "temp_mosfet", lbl("Temp MOSFET"), DOUBLE, "TEMPERATURE", "deg.C"))
    for i in range(temp_count):
        defs.append(SensorDef(p + "temp_%d" % (i + 1), lbl("Temp %d" % (i + 1)), DOUBLE, "TEMPERATURE", "deg.C"))
    for i, sid in enumerate(cell_ids(device)):
        defs.append(SensorDef(sid, lbl("Cell %0*d" % (width, i + 1)), DOUBLE, "VOLTAGE", "V", min_value=lo, max_value=hi))
    defs += [
        SensorDef(p + "cell_delta", lbl("Cell Delta"), DOUBLE, "VOLTAGE", "V"),
        SensorDef(p + "status", lbl("Status"), STRING, "UNKNOWN"),
    ]
    return defs


def temperature_layout(sample: Optional[BmsSample]) -> Tuple[bool, int]:
    """(has MOSFET temperature, number of external probes) delivered by a sample."""
    if sample is None:
        return False, 0
    return sample.mosfet_temperature is not None, len(sample.sensor_temperatures)


def _finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value)


def status_text(sample: BmsSample) -> str:
    """OK | Charging | Discharging | Full, plus error names."""
    state = {
        bl.POWER_SUPPLY_STATUS_CHARGING: "Charging",
        bl.POWER_SUPPLY_STATUS_DISCHARGING: "Discharging",
        bl.POWER_SUPPLY_STATUS_FULL: "Full",
    }.get(bl.power_supply_status(sample.current, sample.soc))
    if sample.vendor in bl.NAMED_ERROR_VENDORS:
        errors = bl.error_names(sample.vendor, sample.problem_code)
    elif sample.problem_code or sample.problem:
        errors = ["problem code 0x%X" % sample.problem_code]
    else:
        errors = []
    parts = ([state] if state else []) + errors
    return ", ".join(parts) or "OK"


def sensor_values(
    device: DeviceConfig, sample: Optional[BmsSample], connected: bool, stale: bool
) -> List[Tuple[str, Value]]:
    """(sensor_id, value) pairs to publish now.

    Only values the sample really contains; None/NaN are skipped. When the
    BMS is disconnected or stale, nothing but the status is published, so no
    old value is sent as if it were current.
    """
    p = id_prefix(device)
    if not connected:
        return [(p + "status", "Disconnected")]
    if stale or sample is None:
        return [(p + "status", "Stale")]

    out: List[Tuple[str, Value]] = []

    def add(suffix: str, value: Optional[float]) -> None:
        if _finite(value):
            out.append((p + suffix, float(value)))

    add("voltage", sample.voltage)
    add("current", sample.current)
    add("soc", sample.soc)
    add("temp_mosfet", sample.mosfet_temperature)
    for i, t in enumerate(sample.sensor_temperatures):
        add("temp_%d" % (i + 1), t)

    cells = [v for v in sample.cell_voltages[: device.cell_count] if _finite(v)]
    for sid, v in zip(cell_ids(device), sample.cell_voltages[: device.cell_count]):
        if _finite(v):
            out.append((sid, float(v)))
    if len(cells) >= 2:
        out.append((p + "cell_delta", max(cells) - min(cells)))

    out.append((p + "status", status_text(sample)))
    return out


# ------------------------------------------------------------------ messages


def info_message(info_cls: Any, sdef: SensorDef) -> Any:
    """xbot_msgs/SensorInfo; unset limits are -1 with their has_* flag False."""
    msg = info_cls()
    msg.sensor_id = sdef.sensor_id
    msg.sensor_name = sdef.name
    msg.value_type = info_cls.TYPE_DOUBLE if sdef.value_type == DOUBLE else info_cls.TYPE_STRING
    msg.value_description = getattr(info_cls, "VALUE_DESCRIPTION_" + sdef.description)
    msg.unit = sdef.unit
    msg.has_min_max = sdef.min_value is not None and sdef.max_value is not None
    msg.min_value = sdef.min_value if msg.has_min_max else UNSET
    msg.max_value = sdef.max_value if msg.has_min_max else UNSET
    msg.has_critical_low = sdef.lower_critical is not None
    msg.lower_critical_value = sdef.lower_critical if msg.has_critical_low else UNSET
    msg.has_critical_high = sdef.upper_critical is not None
    msg.upper_critical_value = sdef.upper_critical if msg.has_critical_high else UNSET
    return msg


def check_ids(defs: Sequence[SensorDef]) -> None:
    ids = [d.sensor_id for d in defs]
    dupes = sorted({i for i in ids if ids.count(i) > 1})
    if dupes:
        raise ValueError("duplicate xbot sensor ids: %s" % ", ".join(dupes))
    bad = [i for i in ids if i.startswith(RESERVED_PREFIX) or not re.match(r"^[a-z0-9_]+$", i)]
    if bad:
        raise ValueError("invalid xbot sensor ids: %s" % ", ".join(bad))


# ------------------------------------------------------------------ publisher


class XbotSensorPublisher:
    """Advertises sensors (info latched) and publishes their data, throttled.

    ``publisher_factory`` has the rospy.Publisher signature
    (topic, msg_class, queue_size=..., latch=...).
    """

    def __init__(
        self,
        devices: Sequence[DeviceConfig],
        publisher_factory: Callable[..., Any],
        info_cls: Any,
        double_cls: Any,
        string_cls: Any,
        rate_hz: float,
    ) -> None:
        self.devices = list(devices)
        self._single = len(self.devices) == 1
        self._factory = publisher_factory
        self._info_cls, self._double_cls, self._string_cls = info_cls, double_cls, string_cls
        self._period = 1.0 / rate_hz
        self._next_due: Optional[float] = None
        self._defs: Dict[str, SensorDef] = {}
        self._pubs: Dict[str, Any] = {}      # sensor_id -> data publisher
        self._info_pubs: Dict[str, Any] = {}  # kept alive: latched info
        check_ids([d for dev in self.devices for d in sensor_defs(dev, self._single, True, 9)])
        for dev in self.devices:
            self._register(sensor_defs(dev, self._single))

    @property
    def sensor_ids(self) -> List[str]:
        return list(self._defs)

    def _register(self, defs: Sequence[SensorDef]) -> List[str]:
        added = []
        for sdef in defs:
            if sdef.sensor_id in self._defs:
                continue
            check_ids(list(self._defs.values()) + [sdef])
            topic = TOPIC_PREFIX + sdef.sensor_id
            info_pub = self._factory(topic + "/info", self._info_cls, queue_size=1, latch=True)
            info_pub.publish(info_message(self._info_cls, sdef))
            data_cls = self._double_cls if sdef.value_type == DOUBLE else self._string_cls
            self._pubs[sdef.sensor_id] = self._factory(topic + "/data", data_cls, queue_size=1)
            self._info_pubs[sdef.sensor_id] = info_pub
            self._defs[sdef.sensor_id] = sdef
            added.append(sdef.sensor_id)
        return added

    def update(self, device: DeviceConfig, sample: Optional[BmsSample]) -> List[str]:
        """Register temperature sensors that appeared in the sample. Returns new ids."""
        has_mosfet, temps = temperature_layout(sample)
        return self._register(sensor_defs(device, self._single, has_mosfet, temps))

    def due(self, now: float) -> bool:
        """True if values should be published now (at most rate_hz on average).

        Fixed schedule instead of "time since last publish": a loop iteration
        that is only slightly early due to jitter still publishes, but the
        average rate cannot exceed rate_hz.
        """
        if self._next_due is not None and now < self._next_due - JITTER_TOLERANCE * self._period:
            return False
        self._next_due = (now if self._next_due is None else self._next_due) + self._period
        if self._next_due <= now:  # after a pause: restart the schedule, no burst of catch-up publishes
            self._next_due = now + self._period
        return True

    def publish(self, device: DeviceConfig, sample: Optional[BmsSample], connected: bool, stale: bool,
                stamp: Any) -> int:
        """Publish current values of one BMS; returns the number of messages sent."""
        sent = 0
        for sensor_id, value in sensor_values(device, sample, connected, stale):
            pub = self._pubs.get(sensor_id)
            if pub is None:  # e.g. a temperature probe that is not registered (yet)
                continue
            msg = (self._string_cls if isinstance(value, str) else self._double_cls)()
            msg.stamp = stamp
            msg.data = value
            pub.publish(msg)
            sent += 1
        return sent
