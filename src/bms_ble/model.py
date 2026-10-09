"""Vendor independent data model for one BMS sample coming from the bridge.

The bridge forwards aiobmsble's ``BMSSample`` dict as JSON. This module turns
it into a typed object and never invents values: anything the library did not
deliver (or delivered as non-finite / wrong type) becomes ``None``.
"""

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

VENDOR_BY_MODULE = {
    "jikong_bms": "jk",
    "jbd_bms": "jbd",
    "ant_bms": "ant",
    "ant_leg_bms": "ant",
}


def _num(value: Any) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _flag(value: Any) -> Optional[bool]:
    return value if isinstance(value, bool) else None


@dataclass
class TempReading:
    value: float
    type: str  # aiobmsble TempSensor.T name: MOSFET, GENERIC, BALANCER, ...


@dataclass
class BmsSample:
    bms_type: str                          # aiobmsble module, e.g. "jikong_bms"
    voltage: Optional[float] = None        # V
    current: Optional[float] = None        # A, + charging / - discharging
    soc: Optional[float] = None            # %
    cell_voltages: List[float] = field(default_factory=list)
    temperatures: List[TempReading] = field(default_factory=list)
    remaining_ah: Optional[float] = None   # aiobmsble "cycle_charge"
    capacity_ah: Optional[float] = None    # aiobmsble "design_capacity" (as reported by the BMS)
    cycles: Optional[int] = None
    soh: Optional[float] = None            # %
    charge_mosfet: Optional[bool] = None
    discharge_mosfet: Optional[bool] = None
    problem: bool = False
    problem_code: int = 0

    @property
    def vendor(self) -> str:
        return VENDOR_BY_MODULE.get(self.bms_type, "other")

    @property
    def mosfet_temperature(self) -> Optional[float]:
        for t in self.temperatures:
            if t.type == "MOSFET":
                return t.value
        return None

    @property
    def max_temperature(self) -> Optional[float]:
        return max((t.value for t in self.temperatures), default=None)

    @property
    def state_temperature(self) -> Optional[float]:
        """BatteryState.temperature: MOSFET temperature (JK), else the hottest sensor."""
        if self.vendor == "jk":
            return self.mosfet_temperature
        return self.max_temperature

    @property
    def sensor_temperatures(self) -> List[float]:
        """External probes (JK: T1, T2, ...; JBD: NTCs, typed CELL by aiobmsble).

        Unplugged probes ("NA") are omitted by aiobmsble. MOSFET and balancer
        temperatures are not probes.
        """
        return [t.value for t in self.temperatures if t.type in ("GENERIC", "CELL")]

    @classmethod
    def from_bridge(cls, bms_type: str, data: Dict[str, Any]) -> "BmsSample":
        temps = []
        for t in data.get("temp_values") or []:
            if isinstance(t, dict) and _num(t.get("value")) is not None:
                temps.append(TempReading(_num(t["value"]), str(t.get("type", "GENERIC"))))
            elif _num(t) is not None:
                temps.append(TempReading(_num(t), "GENERIC"))
        cells = [v for v in (_num(c) for c in data.get("cell_voltages") or []) if v is not None]
        cycles = _num(data.get("cycles"))
        code = data.get("problem_code", 0)
        return cls(
            bms_type=str(bms_type),
            voltage=_num(data.get("voltage")),
            current=_num(data.get("current")),
            soc=_num(data.get("battery_level")),
            cell_voltages=cells,
            temperatures=temps,
            remaining_ah=_num(data.get("cycle_charge")),
            capacity_ah=_num(data.get("design_capacity")),
            cycles=int(cycles) if cycles is not None else None,
            soh=_num(data.get("battery_health")),
            charge_mosfet=_flag(data.get("chrg_mosfet")),
            discharge_mosfet=_flag(data.get("dischrg_mosfet")),
            problem=bool(data.get("problem", False)),
            problem_code=int(code) if isinstance(code, int) and not isinstance(code, bool) else 0,
        )
