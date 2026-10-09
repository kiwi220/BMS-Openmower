"""Conversion of BMS samples to ROS messages (message classes are injected).

No rospy import: the functions take the message class (sensor_msgs
BatteryState, mower_msgs Bms) as a parameter, so they can be unit tested with
simple stand-ins and without hardware.
"""

import json
import math
from typing import Any, List, Optional, Sequence

from . import battery_logic as bl
from .config import DeviceConfig
from .model import BmsSample

NAN = float("nan")


def _or_nan(value: Optional[float]) -> float:
    return NAN if value is None else float(value)


def _finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value)


def battery_state(
    msg_cls: Any,
    device: DeviceConfig,
    sample: Optional[BmsSample],
    connected: bool,
    stale: bool,
    stamp: Any,
    frame_id: str = "battery",
    serial_number: str = "",
) -> Any:
    """BatteryState for one pack. Missing values are NaN (per BatteryState.msg)."""
    msg = msg_cls()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.power_supply_technology = bl.POWER_SUPPLY_TECHNOLOGY_LION
    msg.present = bool(connected)
    msg.design_capacity = float(device.nominal_capacity_ah)
    msg.location = device.name
    msg.serial_number = serial_number

    if sample is None:
        msg.voltage = msg.current = msg.charge = msg.capacity = msg.percentage = msg.temperature = NAN
        msg.cell_voltage = [NAN] * device.cell_count
        msg.power_supply_status = bl.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = bl.POWER_SUPPLY_HEALTH_UNKNOWN
        return msg

    capacity = sample.capacity_ah if _finite(sample.capacity_ah) and sample.capacity_ah > 0 else device.nominal_capacity_ah
    percentage = NAN
    if _finite(sample.soc):
        percentage = min(max(sample.soc / 100.0, 0.0), 1.0)
    if _finite(sample.remaining_ah):
        charge = sample.remaining_ah
    elif math.isfinite(percentage):
        charge = percentage * capacity
    else:
        charge = NAN

    cells = list(sample.cell_voltages[: device.cell_count])
    cells += [NAN] * (device.cell_count - len(cells))

    msg.voltage = _or_nan(sample.voltage)
    msg.current = _or_nan(sample.current)
    msg.capacity = float(capacity)
    msg.charge = float(charge)
    msg.percentage = percentage
    msg.temperature = _or_nan(sample.state_temperature)
    msg.cell_voltage = cells
    msg.power_supply_status = bl.power_supply_status(sample.current, sample.soc)
    msg.power_supply_health = bl.power_supply_health(
        sample.vendor, sample.problem_code, sample.problem, stale, sample.soc
    )
    return msg


def _all_finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(v) for v in values)


def _sum(values: Sequence[float]) -> float:
    return float(sum(values)) if _all_finite(values) else NAN


def _mean(values: Sequence[float]) -> float:
    return float(sum(values) / len(values)) if values and _all_finite(values) else NAN


def _min(values: Sequence[float]) -> float:
    return float(min(values)) if values and _all_finite(values) else NAN


def combined_state(msg_cls: Any, states: List[Any], connection: str, stamp: Any, frame_id: str = "battery") -> Any:
    """Aggregate per-pack BatteryState messages.

    series:   V = sum, I = mean, Ah = min (weakest pack limits), SoC = min
    parallel: V = mean, I = sum, Ah = sum, SoC = charge / capacity
    A value that is NaN in any pack is NaN in the result.
    """
    if connection not in ("series", "parallel"):
        raise ValueError("connection must be 'series' or 'parallel'")
    msg = msg_cls()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.location = "combined"
    msg.power_supply_technology = bl.POWER_SUPPLY_TECHNOLOGY_LION
    msg.present = all(s.present for s in states)

    volts = [s.voltage for s in states]
    amps = [s.current for s in states]
    charge = [s.charge for s in states]
    capacity = [s.capacity for s in states]
    design = [s.design_capacity for s in states]
    temps = [s.temperature for s in states if math.isfinite(s.temperature)]

    if connection == "series":
        msg.voltage, msg.current = _sum(volts), _mean(amps)
        msg.charge, msg.capacity, msg.design_capacity = _min(charge), _min(capacity), _min(design)
        msg.percentage = _min([s.percentage for s in states])
        msg.cell_voltage = [v for s in states for v in s.cell_voltage]
    else:
        msg.voltage, msg.current = _mean(volts), _sum(amps)
        msg.charge, msg.capacity, msg.design_capacity = _sum(charge), _sum(capacity), _sum(design)
        if math.isfinite(msg.charge) and math.isfinite(msg.capacity) and msg.capacity > 0:
            msg.percentage = min(max(msg.charge / msg.capacity, 0.0), 1.0)
        else:
            msg.percentage = NAN
        msg.cell_voltage = []  # cells of parallel packs are not one string

    msg.temperature = max(temps) if temps else NAN
    soc = msg.percentage * 100.0 if math.isfinite(msg.percentage) else None
    msg.power_supply_status = bl.power_supply_status(msg.current, soc)
    msg.power_supply_health = bl.worst_health(s.power_supply_health for s in states)
    return msg


def bms_message(msg_cls: Any, sample: BmsSample, stale: bool, stamp: Any) -> Any:
    """mower_msgs/Bms (as consumed by mower_logic on /ll/bms)."""
    msg = msg_cls()
    msg.stamp = stamp
    if stale:
        # mower_logic keeps the last message forever; NaN makes
        # utils::GetFirstValid() skip it instead of trusting an old voltage.
        msg.voltage = msg.current = msg.relative_state_of_charge = NAN
        msg.remaining_capacity = msg.full_charge_capacity = msg.temperature = NAN
        msg.battery_status = "Stale"
        return msg

    status = []
    if _finite(sample.soc) and sample.soc >= 100:
        status.append("Fully charged")
    if _finite(sample.current) and sample.current < -bl.CURRENT_THRESHOLD_A:
        status.append("Discharging")
    if sample.vendor in bl.NAMED_ERROR_VENDORS:
        status += ["ALARM: %s" % e for e in bl.error_names(sample.vendor, sample.problem_code)]
    elif sample.problem_code or sample.problem:
        status.append("ALARM: problem code 0x%X" % sample.problem_code)

    msg.voltage = _or_nan(sample.voltage)
    msg.current = _or_nan(sample.current)
    msg.relative_state_of_charge = _or_nan(sample.soc)  # percent, SMBus convention
    msg.remaining_capacity = _or_nan(sample.remaining_ah)
    msg.full_charge_capacity = _or_nan(sample.capacity_ah)
    msg.cycle_count = min(sample.cycles or 0, 0xFFFF)
    msg.temperature = _or_nan(sample.state_temperature)
    msg.battery_status = ", ".join(status)
    msg.extra_data = json.dumps({
        "bms_type": sample.bms_type,
        "cells": [round(v, 3) for v in sample.cell_voltages],
        "temperatures": [{"type": t.type, "value": t.value} for t in sample.temperatures],
        "charge_mosfet": sample.charge_mosfet,
        "discharge_mosfet": sample.discharge_mosfet,
        "soh": sample.soh,
        "problem_code": sample.problem_code,
    })
    return msg
