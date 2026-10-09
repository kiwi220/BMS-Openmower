"""Library data (aiobmsble sample) -> BatteryState / Bms conversion tests.

Runs without ROS and without hardware: message classes are simple stand-ins
with the same fields (a test compares against sensor_msgs when available).
"""

import copy
import json
import math
import os
import sys
import unittest
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import fixtures  # noqa: E402
from bms_ble import battery_logic as bl  # noqa: E402
from bms_ble import conversion  # noqa: E402
from bms_ble.config import DeviceConfig  # noqa: E402
from bms_ble.model import BmsSample  # noqa: E402


class FakeBatteryState:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.cell_voltage = []
        self.location = self.serial_number = ""


class FakeBms:
    pass


JK = DeviceConfig("main_pack", "jk", "C8:47:80:00:00:01", cell_count=7, nominal_capacity_ah=10.0, primary=True)
ANT = DeviceConfig("second", "ant", "C8:47:80:00:00:02", cell_count=7, nominal_capacity_ah=10.0)


def jk_sample(**changes):
    data = copy.deepcopy(fixtures.JK_SAMPLE_V11)
    data.update(changes)
    return BmsSample.from_bridge("jikong_bms", data)


def state(sample, device=JK, connected=True, stale=False):
    return conversion.battery_state(FakeBatteryState, device, sample, connected, stale, "stamp")


class ModelTest(unittest.TestCase):
    def test_real_jk_sample(self):
        s = jk_sample()
        self.assertEqual(s.vendor, "jk")
        self.assertAlmostEqual(s.voltage, 51.689)
        self.assertEqual(len(s.cell_voltages), 16)
        self.assertAlmostEqual(s.mosfet_temperature, 17.3)
        self.assertEqual(s.sensor_temperatures[:2], [17.7, 17.7])
        self.assertAlmostEqual(s.remaining_ah, 1.043)
        self.assertEqual(s.capacity_ah, 2)
        self.assertTrue(s.charge_mosfet)

    def test_missing_and_invalid_fields_become_none(self):
        s = BmsSample.from_bridge("ant_bms", {"voltage": float("nan"), "current": "x", "battery_level": True})
        self.assertIsNone(s.voltage)
        self.assertIsNone(s.current)
        self.assertIsNone(s.soc)
        self.assertEqual(s.cell_voltages, [])
        self.assertIsNone(s.state_temperature)
        self.assertIsNone(s.charge_mosfet)

    def test_ant_temperature_is_hottest_sensor(self):
        s = BmsSample.from_bridge("ant_bms", {"temp_values": [
            {"value": 21.0, "type": "GENERIC"}, {"value": 35.5, "type": "MOSFET"}, {"value": 24.0, "type": "BALANCER"},
        ]})
        self.assertEqual(s.vendor, "ant")
        self.assertEqual(s.state_temperature, 35.5)

    def test_jk_temperature_is_mosfet_even_if_not_hottest(self):
        s = jk_sample(temp_values=[{"value": 40.0, "type": "GENERIC"}, {"value": 20.0, "type": "MOSFET"}])
        self.assertEqual(s.state_temperature, 20.0)


class BatteryStateTest(unittest.TestCase):
    def test_scaling_and_fields(self):
        msg = state(jk_sample())
        self.assertAlmostEqual(msg.voltage, 51.689)
        self.assertAlmostEqual(msg.percentage, 0.52)          # 52 % -> 0.52
        self.assertAlmostEqual(msg.charge, 1.043)             # remaining Ah from BMS
        self.assertAlmostEqual(msg.capacity, 2.0)             # capacity reported by BMS
        self.assertAlmostEqual(msg.design_capacity, 10.0)     # nominal from config
        self.assertAlmostEqual(msg.temperature, 17.3)         # JK: MOSFET
        self.assertEqual(len(msg.cell_voltage), 7)            # cell_count from config
        self.assertAlmostEqual(msg.cell_voltage[0], 3.246)
        self.assertTrue(msg.present)
        self.assertEqual(msg.location, "main_pack")
        self.assertEqual(msg.power_supply_technology, bl.POWER_SUPPLY_TECHNOLOGY_LION)
        self.assertEqual(msg.power_supply_status, bl.POWER_SUPPLY_STATUS_NOT_CHARGING)  # 0 A
        self.assertEqual(msg.power_supply_health, bl.POWER_SUPPLY_HEALTH_GOOD)

    def test_current_sign(self):
        self.assertEqual(state(jk_sample(current=2.5)).power_supply_status, bl.POWER_SUPPLY_STATUS_CHARGING)
        dis = state(jk_sample(current=-4.2))
        self.assertAlmostEqual(dis.current, -4.2)
        self.assertEqual(dis.power_supply_status, bl.POWER_SUPPLY_STATUS_DISCHARGING)

    def test_full(self):
        self.assertEqual(state(jk_sample(battery_level=100)).power_supply_status, bl.POWER_SUPPLY_STATUS_FULL)

    def test_fewer_cells_than_configured_padded_with_nan(self):
        msg = state(jk_sample(cell_voltages=[3.3, 3.31]))
        self.assertEqual(msg.cell_voltage[:2], [3.3, 3.31])
        self.assertTrue(all(math.isnan(v) for v in msg.cell_voltage[2:]))
        self.assertEqual(len(msg.cell_voltage), 7)

    def test_missing_values_are_nan_not_invented(self):
        s = BmsSample.from_bridge("ant_bms", {"voltage": 26.1})
        msg = state(s, device=ANT)
        self.assertAlmostEqual(msg.voltage, 26.1)
        for field in ("current", "percentage", "charge", "temperature"):
            self.assertTrue(math.isnan(getattr(msg, field)), field)
        self.assertAlmostEqual(msg.capacity, 10.0)  # falls back to nominal
        self.assertEqual(msg.power_supply_status, bl.POWER_SUPPLY_STATUS_UNKNOWN)

    def test_charge_derived_from_soc_if_remaining_missing(self):
        data = dict(fixtures.JK_SAMPLE_V11)
        del data["cycle_charge"]
        msg = state(BmsSample.from_bridge("jikong_bms", data))
        self.assertAlmostEqual(msg.charge, 0.52 * 2.0)

    def test_no_sample_yet(self):
        msg = state(None, connected=False)
        self.assertFalse(msg.present)
        self.assertTrue(math.isnan(msg.voltage))
        self.assertEqual(len(msg.cell_voltage), 7)
        self.assertEqual(msg.power_supply_health, bl.POWER_SUPPLY_HEALTH_UNKNOWN)

    def test_disconnected_keeps_last_values(self):
        msg = state(jk_sample(), connected=False)
        self.assertFalse(msg.present)
        self.assertAlmostEqual(msg.voltage, 51.689)
        self.assertEqual(msg.power_supply_health, bl.POWER_SUPPLY_HEALTH_GOOD)

    def test_stale_is_unknown_health(self):
        msg = state(jk_sample(), connected=False, stale=True)
        self.assertAlmostEqual(msg.voltage, 51.689)
        self.assertEqual(msg.power_supply_health, bl.POWER_SUPPLY_HEALTH_UNKNOWN)

    def test_jk_error_flags(self):
        self.assertEqual(state(jk_sample(problem_code=1 << 5)).power_supply_health,
                         bl.POWER_SUPPLY_HEALTH_OVERVOLTAGE)
        self.assertEqual(state(jk_sample(problem_code=1 << 12)).power_supply_health, bl.POWER_SUPPLY_HEALTH_DEAD)

    def test_ant_problem_is_unspec_failure(self):
        s = BmsSample.from_bridge("ant_bms", {"voltage": 26.0, "problem_code": 0x200, "problem": True})
        self.assertEqual(state(s, device=ANT).power_supply_health, bl.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)

    def test_soc_clamped(self):
        self.assertEqual(state(jk_sample(battery_level=104)).percentage, 1.0)


class CombinedTest(unittest.TestCase):
    def packs(self):
        a = state(jk_sample(voltage=25.0, current=-2.0, battery_level=80, cycle_charge=8.0, design_capacity=10))
        b = state(jk_sample(voltage=26.0, current=-2.0, battery_level=60, cycle_charge=6.0, design_capacity=10))
        return [a, b]

    def test_series(self):
        m = conversion.combined_state(FakeBatteryState, self.packs(), "series", "stamp")
        self.assertAlmostEqual(m.voltage, 51.0)
        self.assertAlmostEqual(m.current, -2.0)
        self.assertAlmostEqual(m.charge, 6.0)
        self.assertAlmostEqual(m.percentage, 0.6)
        self.assertEqual(len(m.cell_voltage), 14)
        self.assertEqual(m.power_supply_status, bl.POWER_SUPPLY_STATUS_DISCHARGING)

    def test_parallel(self):
        m = conversion.combined_state(FakeBatteryState, self.packs(), "parallel", "stamp")
        self.assertAlmostEqual(m.voltage, 25.5)
        self.assertAlmostEqual(m.current, -4.0)
        self.assertAlmostEqual(m.capacity, 20.0)
        self.assertAlmostEqual(m.percentage, 0.7)  # 14 Ah / 20 Ah
        self.assertEqual(m.cell_voltage, [])

    def test_worst_health_and_presence(self):
        a, b = self.packs()
        b.present = False
        b.power_supply_health = bl.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        m = conversion.combined_state(FakeBatteryState, [a, b], "series", "stamp")
        self.assertFalse(m.present)
        self.assertEqual(m.power_supply_health, bl.POWER_SUPPLY_HEALTH_OVERVOLTAGE)

    def test_nan_propagates(self):
        a, b = self.packs()
        b.voltage = float("nan")
        self.assertTrue(math.isnan(conversion.combined_state(FakeBatteryState, [a, b], "series", "s").voltage))

    def test_invalid_connection(self):
        with self.assertRaises(ValueError):
            conversion.combined_state(FakeBatteryState, self.packs(), "", "s")


class BmsMessageTest(unittest.TestCase):
    def test_fields(self):
        m = conversion.bms_message(FakeBms, jk_sample(current=-3.0, problem_code=1 << 5), False, "stamp")
        self.assertAlmostEqual(m.voltage, 51.689)
        self.assertEqual(m.relative_state_of_charge, 52.0)
        self.assertEqual(m.temperature, 17.3)
        self.assertIn("Discharging", m.battery_status)
        self.assertIn("ALARM: Battery pack overvoltage", m.battery_status)
        self.assertEqual(len(json.loads(m.extra_data)["cells"]), 16)

    def test_stale_sends_nan(self):
        m = conversion.bms_message(FakeBms, jk_sample(), True, "stamp")
        self.assertTrue(math.isnan(m.voltage))
        self.assertEqual(m.battery_status, "Stale")


class RosMessageTest(unittest.TestCase):
    def test_real_battery_state(self):
        try:
            from sensor_msgs.msg import BatteryState
        except ImportError:
            self.skipTest("sensor_msgs not available")
        msg = conversion.battery_state(BatteryState, JK, jk_sample(), True, False, None)
        msg.header.stamp = __import__("rospy").Time(0)
        buf = __import__("io").BytesIO()
        msg.serialize(buf)  # type/field check of the real message


JBD = DeviceConfig("pack_jbd", "jbd", "A5:C2:37:00:00:01", cell_count=4, nominal_capacity_ah=5.0)


def jbd_sample(base=None, **changes):
    data = copy.deepcopy(base or fixtures.JBD_SAMPLE_4S)
    data.update(changes)
    return BmsSample.from_bridge("jbd_bms", data)


class JbdTest(unittest.TestCase):
    def test_model(self):
        s = jbd_sample()
        self.assertEqual(s.vendor, "jbd")
        self.assertIsNone(s.mosfet_temperature)                      # aiobmsble reports none for JBD
        self.assertEqual(s.sensor_temperatures, [22.4, 22.3, 21.7])  # aiobmsble types them CELL
        self.assertEqual(s.state_temperature, 22.4)                   # hottest available
        self.assertEqual(len(s.cell_voltages), 4)
        self.assertTrue(s.charge_mosfet)

    def test_probe_types(self):
        # CELL counts as external probe, MOSFET and BALANCER do not
        s = BmsSample.from_bridge("ant_bms", {"temp_values": [
            {"value": 20.0, "type": "GENERIC"}, {"value": 21.0, "type": "CELL"},
            {"value": 30.0, "type": "MOSFET"}, {"value": 25.0, "type": "BALANCER"}]})
        self.assertEqual(s.sensor_temperatures, [20.0, 21.0])
        self.assertEqual(s.state_temperature, 30.0)   # hottest of all, as before for ANT

    def test_battery_state(self):
        msg = state(jbd_sample(), device=JBD)
        self.assertAlmostEqual(msg.voltage, 15.60)
        self.assertAlmostEqual(msg.current, 0.0)
        self.assertAlmostEqual(msg.percentage, 1.0)
        self.assertAlmostEqual(msg.charge, 4.98)        # remaining Ah from the BMS
        self.assertAlmostEqual(msg.capacity, 5.0)       # capacity reported by the BMS
        self.assertAlmostEqual(msg.design_capacity, 5.0)
        self.assertAlmostEqual(msg.temperature, 22.4)
        self.assertEqual(msg.cell_voltage, [3.909, 3.901, 3.895, 3.901])
        self.assertEqual(msg.power_supply_status, bl.POWER_SUPPLY_STATUS_FULL)   # 100 %, no current
        self.assertEqual(msg.power_supply_health, bl.POWER_SUPPLY_HEALTH_GOOD)
        self.assertTrue(msg.present)

    def test_charging_and_discharging_signs(self):
        self.assertEqual(state(jbd_sample(current=3.0, battery_level=80), device=JBD).power_supply_status,
                         bl.POWER_SUPPLY_STATUS_CHARGING)
        dis = state(jbd_sample(current=-2.5, battery_level=80), device=JBD)
        self.assertAlmostEqual(dis.current, -2.5)
        self.assertEqual(dis.power_supply_status, bl.POWER_SUPPLY_STATUS_DISCHARGING)

    def test_real_cell_overvoltage_capture_at_full_is_informational(self):
        # real capture: 100 %, cell overvoltage protection ended the charge
        s = jbd_sample(fixtures.JBD_SAMPLE_CELL_OVERVOLTAGE)
        msg = state(s, device=DeviceConfig("big", "jbd", "A5:C2:37:00:00:02", 4, 280.0))
        self.assertEqual(msg.power_supply_health, bl.POWER_SUPPLY_HEALTH_GOOD)
        self.assertEqual(msg.power_supply_status, bl.POWER_SUPPLY_STATUS_FULL)
        self.assertAlmostEqual(msg.voltage, 14.28)
        # still listed, like JK's "Battery is fully charged" bit
        bms = conversion.bms_message(FakeBms, s, False, "stamp")
        self.assertIn("ALARM: Cell overvoltage", bms.battery_status)
        self.assertEqual(json.loads(bms.extra_data)["problem_code"], 1)

    def test_cell_overvoltage_below_full_stays_overvoltage(self):
        s = jbd_sample(fixtures.JBD_SAMPLE_CELL_OVERVOLTAGE, battery_level=90)
        self.assertEqual(state(s, device=JBD).power_supply_health, bl.POWER_SUPPLY_HEALTH_OVERVOLTAGE)
        no_soc = jbd_sample(fixtures.JBD_SAMPLE_CELL_OVERVOLTAGE, battery_level=None)
        self.assertEqual(state(no_soc, device=JBD).power_supply_health, bl.POWER_SUPPLY_HEALTH_OVERVOLTAGE)

    def test_stale(self):
        msg = state(jbd_sample(fixtures.JBD_SAMPLE_CELL_OVERVOLTAGE), device=JBD, connected=False, stale=True)
        self.assertEqual(msg.power_supply_health, bl.POWER_SUPPLY_HEALTH_UNKNOWN)

    def test_problem_flag_without_code_gives_no_alarm(self):
        # aiobmsble sets "problem" for sanity checks too; same behaviour as for JK
        for s in (jbd_sample(problem=True, problem_code=0), jk_sample(problem=True, problem_code=0)):
            self.assertNotIn("ALARM", conversion.bms_message(FakeBms, s, False, "stamp").battery_status)
            self.assertEqual(state(s).power_supply_health, bl.POWER_SUPPLY_HEALTH_GOOD)


if __name__ == "__main__":
    unittest.main()
