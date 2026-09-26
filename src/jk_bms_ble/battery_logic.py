"""Mapping of JK-BMS data to sensor_msgs/BatteryState status and health values.

Kept free of ROS imports so it can be unit tested without a ROS install. The
constants mirror sensor_msgs/BatteryState.msg (ROS 1 Noetic); a test checks
them against the real message when it is importable.
"""

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


def _mask(*bits: int) -> int:
    value = 0
    for bit in bits:
        value |= 1 << bit
    return value


# JK error bits (see protocol.ERROR_NAMES) grouped by BatteryState health, in
# priority order. Bits not listed (wire resistance, "fully charged", GPS,
# password reminder, ...) are informational and keep the health GOOD.
HEALTH_ERROR_GROUPS = (
    (POWER_SUPPLY_HEALTH_DEAD, _mask(11, 12)),                 # cell / pack undervoltage
    (POWER_SUPPLY_HEALTH_OVERVOLTAGE, _mask(5)),               # pack overvoltage
    (POWER_SUPPLY_HEALTH_OVERHEAT, _mask(1, 8, 15, 21)),       # MOSFET / charge / discharge / battery
    (POWER_SUPPLY_HEALTH_COLD, _mask(9, 27)),                  # charge / discharge undertemperature
    (POWER_SUPPLY_HEALTH_UNSPEC_FAILURE,
     _mask(2, 6, 7, 10, 13, 14, 16, 17, 20, 22, 24, 25, 26)),  # overcurrent, short circuit, MOSFET fault, ...
)


def power_supply_status(current: float, state_of_charge: float) -> int:
    """Charging state from current (A, + = charging) and SoC (%).

    FULL wins over a small (balancing / trickle) charge current at 100 % SoC,
    but a real discharge is always reported as DISCHARGING.
    """
    if current < -CURRENT_THRESHOLD_A:
        return POWER_SUPPLY_STATUS_DISCHARGING
    if state_of_charge >= 100:
        return POWER_SUPPLY_STATUS_FULL
    if current > CURRENT_THRESHOLD_A:
        return POWER_SUPPLY_STATUS_CHARGING
    return POWER_SUPPLY_STATUS_NOT_CHARGING


def power_supply_health(error_bitmask: int, stale: bool = False) -> int:
    """Health from the JK error bitmask; UNKNOWN when the data is stale."""
    if stale:
        return POWER_SUPPLY_HEALTH_UNKNOWN
    for health, mask in HEALTH_ERROR_GROUPS:
        if error_bitmask & mask:
            return health
    return POWER_SUPPLY_HEALTH_GOOD
