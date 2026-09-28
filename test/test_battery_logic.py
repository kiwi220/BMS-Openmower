"""Status / health mapping tests."""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from bms_ble import battery_logic as bl  # noqa: E402


class StatusTest(unittest.TestCase):
    def test_charging(self):
        self.assertEqual(bl.power_supply_status(0.5, 80), bl.POWER_SUPPLY_STATUS_CHARGING)

    def test_discharging(self):
        self.assertEqual(bl.power_supply_status(-3.0, 80), bl.POWER_SUPPLY_STATUS_DISCHARGING)

    def test_full(self):
        self.assertEqual(bl.power_supply_status(0.0, 100), bl.POWER_SUPPLY_STATUS_FULL)
        self.assertEqual(bl.power_supply_status(0.3, 100), bl.POWER_SUPPLY_STATUS_FULL)

    def test_discharge_at_full_soc(self):
        self.assertEqual(bl.power_supply_status(-1.0, 100), bl.POWER_SUPPLY_STATUS_DISCHARGING)

    def test_idle_deadband(self):
        for current in (0.0, 0.1, -0.1, 0.05):
            self.assertEqual(bl.power_supply_status(current, 50), bl.POWER_SUPPLY_STATUS_NOT_CHARGING)

    def test_missing_values(self):
        self.assertEqual(bl.power_supply_status(None, 50), bl.POWER_SUPPLY_STATUS_UNKNOWN)
        self.assertEqual(bl.power_supply_status(float("nan"), None), bl.POWER_SUPPLY_STATUS_UNKNOWN)
        self.assertEqual(bl.power_supply_status(None, 100), bl.POWER_SUPPLY_STATUS_FULL)
        self.assertEqual(bl.power_supply_status(1.0, None), bl.POWER_SUPPLY_STATUS_CHARGING)


class HealthTest(unittest.TestCase):
    def jk(self, code, stale=False):
        return bl.power_supply_health("jk", code, bool(code), stale)

    def test_good(self):
        self.assertEqual(self.jk(0), bl.POWER_SUPPLY_HEALTH_GOOD)

    def test_informational_bits_stay_good(self):
        self.assertEqual(self.jk(1 << 0 | 1 << 4), bl.POWER_SUPPLY_HEALTH_GOOD)

    def test_overvoltage(self):
        self.assertEqual(self.jk(1 << 5), bl.POWER_SUPPLY_HEALTH_OVERVOLTAGE)

    def test_undervoltage_is_dead(self):
        self.assertEqual(self.jk(1 << 11), bl.POWER_SUPPLY_HEALTH_DEAD)
        self.assertEqual(self.jk(1 << 12), bl.POWER_SUPPLY_HEALTH_DEAD)

    def test_dead_has_priority(self):
        self.assertEqual(self.jk(1 << 5 | 1 << 12), bl.POWER_SUPPLY_HEALTH_DEAD)

    def test_overheat_cold_failure(self):
        self.assertEqual(self.jk(1 << 1), bl.POWER_SUPPLY_HEALTH_OVERHEAT)
        self.assertEqual(self.jk(1 << 9), bl.POWER_SUPPLY_HEALTH_COLD)
        self.assertEqual(self.jk(1 << 14), bl.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)

    def test_stale_is_unknown(self):
        self.assertEqual(self.jk(0, stale=True), bl.POWER_SUPPLY_HEALTH_UNKNOWN)
        self.assertEqual(self.jk(1 << 5, stale=True), bl.POWER_SUPPLY_HEALTH_UNKNOWN)

    def test_other_vendors(self):
        self.assertEqual(bl.power_supply_health("ant", 0, False), bl.POWER_SUPPLY_HEALTH_GOOD)
        self.assertEqual(bl.power_supply_health("ant", 0x2, True), bl.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
        self.assertEqual(bl.power_supply_health("ant", 0, True), bl.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)

    def test_jk_error_names(self):
        self.assertEqual(bl.jk_error_names(1 << 5 | 1 << 12), ["Battery pack overvoltage", "Battery pack undervoltage"])

    def test_worst_health(self):
        self.assertEqual(
            bl.worst_health([bl.POWER_SUPPLY_HEALTH_GOOD, bl.POWER_SUPPLY_HEALTH_OVERHEAT]),
            bl.POWER_SUPPLY_HEALTH_OVERHEAT,
        )
        self.assertEqual(
            bl.worst_health([bl.POWER_SUPPLY_HEALTH_DEAD, bl.POWER_SUPPLY_HEALTH_UNKNOWN]),
            bl.POWER_SUPPLY_HEALTH_UNKNOWN,
        )


class RosConstantsTest(unittest.TestCase):
    def test_constants_match_sensor_msgs(self):
        try:
            from sensor_msgs.msg import BatteryState
        except ImportError:
            self.skipTest("sensor_msgs not available")
        for name in dir(bl):
            if name.startswith("POWER_SUPPLY_"):
                self.assertEqual(getattr(bl, name), getattr(BatteryState, name), name)


if __name__ == "__main__":
    unittest.main()
