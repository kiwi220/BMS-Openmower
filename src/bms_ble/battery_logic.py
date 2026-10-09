"""Mapping of BMS data to sensor_msgs/BatteryState status and health values.

Kept free of ROS imports so it can be unit tested without a ROS install. The
constants mirror sensor_msgs/BatteryState.msg (ROS 1 Noetic); a test checks
them against the real message when it is importable.
"""

import math
from typing import Iterable, List, Optional

POWER_SUPPLY_STATUS_UNKNOWN = 0
POWER_SUPPLY_STATUS_CHARGING = 1
POWER_SUPPLY_STATUS_DISCHARGING = 2
POWER_SUPPLY_STATUS_NOT_CHARGING = 3
POWER_SUPPLY_STATUS_FULL = 4

POWER_SUPPLY_HEALTH_UNKNOWN = 0
POWER_SUPPLY_HEALTH_GOOD = 1
POWER_SUPPLY_HEALTH_OVERHEAT = 2
POWER_SUPPLY_HEALTH_DEAD = 3
POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
POWER_SUPPLY_HEALTH_COLD = 6

POWER_SUPPLY_TECHNOLOGY_LION = 2

CURRENT_THRESHOLD_A = 0.1

# JK error bitmask (JK02), index = bit number; from esphome-jk-bms. aiobmsble
# reports the lower 16 bits as problem_code for JK02_32S devices.
JK_ERROR_NAMES = (
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
)


def _mask(*bits: int) -> int:
    value = 0
    for bit in bits:
        value |= 1 << bit
    return value


# JK error bits grouped by BatteryState health, in priority order. Bits not
# listed (wire resistance, "fully charged") are informational -> GOOD.
JK_HEALTH_GROUPS = (
    (POWER_SUPPLY_HEALTH_DEAD, _mask(11, 12)),                 # cell / pack undervoltage
    (POWER_SUPPLY_HEALTH_OVERVOLTAGE, _mask(5)),               # pack overvoltage
    (POWER_SUPPLY_HEALTH_OVERHEAT, _mask(1, 8, 15)),           # MOSFET / charge / discharge
    (POWER_SUPPLY_HEALTH_COLD, _mask(9)),                      # charge undertemperature
    (POWER_SUPPLY_HEALTH_UNSPEC_FAILURE, _mask(2, 6, 7, 10, 13, 14)),
)

# JBD protection status (register 0x03, "problem_code" in aiobmsble is the raw
# 16 bit value), index = bit number; from esphome-jbd-bms.
JBD_ERROR_NAMES = (
    "Cell overvoltage",               # bit 0
    "Cell undervoltage",              # bit 1
    "Pack overvoltage",               # bit 2
    "Pack undervoltage",              # bit 3
    "Charging over temperature",      # bit 4
    "Charging under temperature",     # bit 5
    "Discharging over temperature",   # bit 6
    "Discharging under temperature",  # bit 7
    "Charging overcurrent",           # bit 8
    "Discharging overcurrent",        # bit 9
    "Short circuit",                  # bit 10
    "IC front-end error",             # bit 11
    "MOSFET software lock",           # bit 12
    "Charge timeout close",           # bit 13
)

# JBD bits by BatteryState health, priority order. Every JBD protection bit
# means a protection is active, so any other set bit (including the
# undocumented bits 14/15) is reported as UNSPEC_FAILURE rather than ignored.
JBD_HEALTH_GROUPS = (
    (POWER_SUPPLY_HEALTH_DEAD, _mask(1, 3)),                   # cell / pack undervoltage
    (POWER_SUPPLY_HEALTH_OVERVOLTAGE, _mask(0, 2)),            # cell / pack overvoltage
    (POWER_SUPPLY_HEALTH_OVERHEAT, _mask(4, 6)),               # charge / discharge over temperature
    (POWER_SUPPLY_HEALTH_COLD, _mask(5, 7)),                   # charge / discharge under temperature
)
JBD_PROTECTION_MASK = 0xFFFF
# Cell overvoltage protection at 100 % SoC means the charge ended through this
# protection (real capture: 100 %, protection active, charge MOSFET off), so
# like JK's "Battery is fully charged" it is informational then. Below 100 %
# (or without SoC) it stays OVERVOLTAGE: a cell hit the limit before the pack counts as full.
JBD_INFO_WHEN_FULL = _mask(0)

# Severity used when several packs are combined (worst wins)
HEALTH_SEVERITY = {
    POWER_SUPPLY_HEALTH_GOOD: 0,
    POWER_SUPPLY_HEALTH_COLD: 1,
    POWER_SUPPLY_HEALTH_OVERHEAT: 2,
    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: 3,
    POWER_SUPPLY_HEALTH_OVERVOLTAGE: 4,
    POWER_SUPPLY_HEALTH_DEAD: 5,
    POWER_SUPPLY_HEALTH_UNKNOWN: 6,
}


def jk_error_names(bitmask: int) -> List[str]:
    names = []
    for bit in range(32):
        if bitmask & (1 << bit):
            name = JK_ERROR_NAMES[bit] if bit < len(JK_ERROR_NAMES) else ""
            names.append(name or "Unknown error bit %d" % bit)
    return names


def jbd_error_names(bitmask: int) -> List[str]:
    names = []
    for bit in range(16):
        if bitmask & (1 << bit):
            names.append(JBD_ERROR_NAMES[bit] if bit < len(JBD_ERROR_NAMES) else "Unknown error bit %d" % bit)
    return names


NAMED_ERROR_VENDORS = ("jk", "jbd")  # vendors whose problem_code bit layout is documented


def error_names(vendor: str, problem_code: int) -> List[str]:
    """Readable names of the set error bits; empty for vendors whose code layout is not known."""
    if vendor == "jk":
        return jk_error_names(problem_code)
    if vendor == "jbd":
        return jbd_error_names(problem_code)
    return []


def power_supply_status(current: Optional[float], state_of_charge: Optional[float]) -> int:
    """Charging state from current (A, + = charging) and SoC (%).

    FULL wins over a small (balancing / trickle) charge current at 100 % SoC,
    but a real discharge is always reported as DISCHARGING. Missing values
    (None / NaN) are never guessed.
    """
    has_current = current is not None and math.isfinite(current)
    has_soc = state_of_charge is not None and math.isfinite(state_of_charge)
    if has_current and current < -CURRENT_THRESHOLD_A:
        return POWER_SUPPLY_STATUS_DISCHARGING
    if has_soc and state_of_charge >= 100:
        return POWER_SUPPLY_STATUS_FULL
    if not has_current:
        return POWER_SUPPLY_STATUS_UNKNOWN
    if current > CURRENT_THRESHOLD_A:
        return POWER_SUPPLY_STATUS_CHARGING
    return POWER_SUPPLY_STATUS_NOT_CHARGING


def jk_health(error_bitmask: int) -> int:
    for health, mask in JK_HEALTH_GROUPS:
        if error_bitmask & mask:
            return health
    return POWER_SUPPLY_HEALTH_GOOD


def jbd_health(protection_status: int, state_of_charge: Optional[float] = None) -> int:
    if state_of_charge is not None and math.isfinite(state_of_charge) and state_of_charge >= 100:
        protection_status &= ~JBD_INFO_WHEN_FULL
    for health, mask in JBD_HEALTH_GROUPS:
        if protection_status & mask:
            return health
    if protection_status & JBD_PROTECTION_MASK:
        return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE  # overcurrent, short circuit, IC error, lock, timeout, unknown
    return POWER_SUPPLY_HEALTH_GOOD


def power_supply_health(
    vendor: str, problem_code: int, problem: bool, stale: bool = False, state_of_charge: Optional[float] = None
) -> int:
    """Health of one pack; UNKNOWN when the data is stale.

    JK, JBD: problem_code is the vendor's error / protection bitmask ->
    DEAD / OVERVOLTAGE / OVERHEAT / COLD / UNSPEC_FAILURE. For JBD the cell
    overvoltage bit is informational at 100 % SoC (state_of_charge, %).
    Others (ANT, ...): aiobmsble packs vendor specific status codes into
    problem_code whose byte layout is not documented consistently, so any
    reported problem is mapped to UNSPEC_FAILURE rather than guessed.
    """
    if stale:
        return POWER_SUPPLY_HEALTH_UNKNOWN
    if vendor == "jk":
        return jk_health(problem_code)
    if vendor == "jbd":
        return jbd_health(problem_code, state_of_charge)
    if problem_code or problem:
        return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
    return POWER_SUPPLY_HEALTH_GOOD


def worst_health(healths: Iterable[int]) -> int:
    return max(healths, key=lambda h: HEALTH_SEVERITY.get(h, 6), default=POWER_SUPPLY_HEALTH_UNKNOWN)
