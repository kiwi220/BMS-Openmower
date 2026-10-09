"""xbot_monitoring sensor tests (no ROS needed; message classes are stand-ins)."""

import copy
import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import fixtures  # noqa: E402
from bms_ble import xbot_sensors as xs  # noqa: E402
from bms_ble.config import DeviceConfig, parse_xbot_rate, parse_xbot_sensor_bms  # noqa: E402
from bms_ble.model import BmsSample  # noqa: E402


class FakeSensorInfo:
    # same constants as xbot_msgs/SensorInfo.msg (checked against the real one below)
    TYPE_STRING = 1
    TYPE_DOUBLE = 2
    VALUE_DESCRIPTION_UNKNOWN = 0
    VALUE_DESCRIPTION_TEMPERATURE = 1
    VALUE_DESCRIPTION_VELOCITY = 2
    VALUE_DESCRIPTION_ACCELERATION = 3
    VALUE_DESCRIPTION_VOLTAGE = 4
    VALUE_DESCRIPTION_CURRENT = 5
    VALUE_DESCRIPTION_PERCENT = 6
    VALUE_DESCRIPTION_DISTANCE = 7
    VALUE_DESCRIPTION_RPM = 8


class FakeDouble:
    pass


class FakeString:
    pass


class FakePublisher:
    def __init__(self, topic, msg_cls, queue_size=None, latch=False):
        self.topic, self.msg_cls, self.queue_size, self.latch = topic, msg_cls, queue_size, latch
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class Factory:
    def __init__(self):
        self.pubs = {}

    def __call__(self, topic, msg_cls, queue_size=None, latch=False):
        pub = FakePublisher(topic, msg_cls, queue_size, latch)
        self.pubs[topic] = pub
        return pub

    def data(self, sensor_id):
        return [m.data for m in self.pubs[xs.TOPIC_PREFIX + sensor_id + "/data"].msgs]


MAIN = DeviceConfig("main_pack", "jk", "C8:47:80:00:00:01", cell_count=7, nominal_capacity_ah=10.0, primary=True)
SECOND = DeviceConfig("Second", "ant", "C8:47:80:00:00:02", cell_count=7, nominal_capacity_ah=10.0)


def jk_sample(**changes):
    data = copy.deepcopy(fixtures.JK_SAMPLE_V11)
    data.update(changes)
    return BmsSample.from_bridge("jikong_bms", data)


def ids(values):
    return [sid for sid, _ in values]


class IdSchemaTest(unittest.TestCase):
    def test_sanitize(self):
        self.assertEqual(xs.sanitize("Main_Pack"), "main_pack")
        self.assertEqual(xs.sanitize("Akku 1 (vorne)!"), "akku_1_vorne")
        self.assertEqual(xs.sanitize("__A--B__"), "a_b")
        with self.assertRaises(ValueError):
            xs.sanitize("!!!")

    def test_id_scheme_and_names(self):
        defs = xs.sensor_defs(MAIN, single=True)
        self.assertEqual(defs[0].sensor_id, "bms_main_pack_voltage")
        self.assertEqual(defs[0].name, "BMS Voltage")                     # single BMS: no name
        multi = xs.sensor_defs(SECOND, single=False)
        self.assertEqual(multi[0].sensor_id, "bms_second_voltage")        # lower case
        self.assertEqual(multi[0].name, "BMS Second Voltage")

    def test_no_reserved_prefix_and_valid_chars(self):
        for dev in (MAIN, SECOND, DeviceConfig("om_pack", "jk", "C8:47:80:00:00:03", 7, 10.0)):
            for d in xs.sensor_defs(dev, False, True, 3):
                self.assertFalse(d.sensor_id.startswith("om_"), d.sensor_id)
                self.assertRegex(d.sensor_id, r"^[a-z0-9_]+$")

    def test_check_ids_rejects_reserved(self):
        with self.assertRaises(ValueError):
            xs.check_ids([xs.SensorDef("om_v_battery", "x", xs.DOUBLE, "VOLTAGE")])


class SensorListTest(unittest.TestCase):
    def test_order_seven_cells_without_temps(self):
        got = [d.sensor_id[len("bms_main_pack_"):] for d in xs.sensor_defs(MAIN, True)]
        self.assertEqual(
            got,
            ["voltage", "current", "soc"] + ["cell_%02d" % i for i in range(1, 8)] + ["cell_delta", "status"],
        )

    def test_order_with_temps(self):
        got = [d.sensor_id[len("bms_main_pack_"):] for d in xs.sensor_defs(MAIN, True, True, 2)]
        self.assertEqual(got[:6], ["voltage", "current", "soc", "temp_mosfet", "temp_1", "temp_2"])
        self.assertEqual(len(got), 3 + 3 + 7 + 2)

    def test_units_types_and_limits(self):
        defs = {d.sensor_id: d for d in xs.sensor_defs(MAIN, True, True, 1)}
        info = xs.info_message(FakeSensorInfo, defs["bms_main_pack_temp_mosfet"])
        self.assertEqual(info.unit, "deg.C")
        self.assertEqual(info.value_description, FakeSensorInfo.VALUE_DESCRIPTION_TEMPERATURE)
        self.assertEqual(info.value_type, FakeSensorInfo.TYPE_DOUBLE)
        self.assertFalse(info.has_min_max)
        self.assertEqual((info.min_value, info.max_value), (-1.0, -1.0))
        self.assertFalse(info.has_critical_low)
        self.assertFalse(info.has_critical_high)
        self.assertEqual((info.lower_critical_value, info.upper_critical_value), (-1.0, -1.0))

        cell = xs.info_message(FakeSensorInfo, defs["bms_main_pack_cell_01"])
        self.assertTrue(cell.has_min_max)
        self.assertEqual((cell.min_value, cell.max_value), (2.5, 4.2))
        self.assertFalse(cell.has_critical_low)  # no invented safety limits

        status = xs.info_message(FakeSensorInfo, defs["bms_main_pack_status"])
        self.assertEqual(status.value_type, FakeSensorInfo.TYPE_STRING)
        self.assertEqual(status.value_description, FakeSensorInfo.VALUE_DESCRIPTION_UNKNOWN)

    def test_all_ids_unique_with_two_bms(self):
        pub = xs.XbotSensorPublisher([MAIN, SECOND], Factory(), FakeSensorInfo, FakeDouble, FakeString, 1.0)
        pub.update(MAIN, jk_sample())
        pub.update(SECOND, jk_sample())
        self.assertEqual(len(pub.sensor_ids), len(set(pub.sensor_ids)))
        self.assertTrue(any(i.startswith("bms_main_pack_") for i in pub.sensor_ids))
        self.assertTrue(any(i.startswith("bms_second_") for i in pub.sensor_ids))

    def test_colliding_names_rejected(self):
        a = DeviceConfig("Pack", "jk", "C8:47:80:00:00:01", 7, 10.0)
        b = DeviceConfig("pack", "jk", "C8:47:80:00:00:02", 7, 10.0)
        with self.assertRaises(ValueError):
            xs.XbotSensorPublisher([a, b], Factory(), FakeSensorInfo, FakeDouble, FakeString, 1.0)


class ValuesTest(unittest.TestCase):
    def test_real_jk_sample(self):
        values = dict(xs.sensor_values(MAIN, jk_sample(), connected=True, stale=False))
        self.assertAlmostEqual(values["bms_main_pack_voltage"], 51.689)
        self.assertEqual(values["bms_main_pack_soc"], 52.0)
        self.assertEqual(values["bms_main_pack_temp_mosfet"], 17.3)
        self.assertEqual(values["bms_main_pack_temp_1"], 17.7)
        self.assertIn("bms_main_pack_cell_07", values)
        self.assertNotIn("bms_main_pack_cell_08", values)  # only cell_count cells
        self.assertEqual(values["bms_main_pack_status"], "OK")

    def test_missing_values_not_published(self):
        s = BmsSample.from_bridge("ant_bms", {"voltage": 26.0, "current": float("nan")})
        got = ids(xs.sensor_values(MAIN, s, True, False))
        self.assertEqual(got, ["bms_main_pack_voltage", "bms_main_pack_status"])

    def test_cell_delta_with_fewer_cells(self):
        s = jk_sample(cell_voltages=[3.30, 3.35, 3.28])
        values = dict(xs.sensor_values(MAIN, s, True, False))
        self.assertAlmostEqual(values["bms_main_pack_cell_delta"], 0.07)
        self.assertNotIn("bms_main_pack_cell_04", values)
        one = dict(xs.sensor_values(MAIN, jk_sample(cell_voltages=[3.3]), True, False))
        self.assertNotIn("bms_main_pack_cell_delta", one)  # not meaningful with one cell

    def test_cell_delta_limited_to_cell_count(self):
        cells = [3.30] * 7 + [2.00]  # 8th cell is outside cell_count
        values = dict(xs.sensor_values(MAIN, jk_sample(cell_voltages=cells), True, False))
        self.assertAlmostEqual(values["bms_main_pack_cell_delta"], 0.0)

    def test_disconnected_and_stale_only_status(self):
        self.assertEqual(xs.sensor_values(MAIN, jk_sample(), False, False),
                         [("bms_main_pack_status", "Disconnected")])
        self.assertEqual(xs.sensor_values(MAIN, jk_sample(), True, True), [("bms_main_pack_status", "Stale")])
        self.assertEqual(xs.sensor_values(MAIN, None, True, True), [("bms_main_pack_status", "Stale")])

    def test_status_texts(self):
        self.assertEqual(xs.status_text(jk_sample(current=2.0)), "Charging")
        self.assertEqual(xs.status_text(jk_sample(current=-2.0)), "Discharging")
        self.assertEqual(xs.status_text(jk_sample(battery_level=100)), "Full")
        self.assertEqual(xs.status_text(jk_sample(current=-2.0, problem_code=1 << 12)),
                         "Discharging, Battery pack undervoltage")
        self.assertEqual(xs.status_text(jk_sample(problem_code=1 << 5)), "Battery pack overvoltage")
        ant = BmsSample.from_bridge("ant_bms", {"current": 0.0, "problem_code": 0x203, "problem": True})
        self.assertEqual(xs.status_text(ant), "problem code 0x203")


class PublisherTest(unittest.TestCase):
    def make(self, devices=(MAIN,), rate=1.0):
        factory = Factory()
        return xs.XbotSensorPublisher(list(devices), factory, FakeSensorInfo, FakeDouble, FakeString, rate), factory

    def test_topics_absolute_and_info_latched(self):
        pub, factory = self.make()
        info = factory.pubs["/xbot_monitoring/sensors/bms_main_pack_voltage/info"]
        self.assertTrue(info.latch)
        self.assertEqual(info.queue_size, 1)
        self.assertEqual(len(info.msgs), 1)
        self.assertEqual(info.msgs[0].sensor_id, "bms_main_pack_voltage")
        self.assertIs(factory.pubs["/xbot_monitoring/sensors/bms_main_pack_status/data"].msg_cls, FakeString)
        self.assertTrue(all(t.startswith("/xbot_monitoring/sensors/bms_") for t in factory.pubs))

    def test_temperature_sensors_registered_late(self):
        pub, factory = self.make()
        self.assertNotIn("bms_main_pack_temp_mosfet", pub.sensor_ids)
        added = pub.update(MAIN, jk_sample())
        self.assertEqual(added[:3], ["bms_main_pack_temp_mosfet", "bms_main_pack_temp_1", "bms_main_pack_temp_2"])
        self.assertTrue(factory.pubs["/xbot_monitoring/sensors/bms_main_pack_temp_1/info"].latch)
        self.assertEqual(pub.update(MAIN, jk_sample()), [])  # nothing new, no re-advertising

    def test_publish_values_and_stale(self):
        pub, factory = self.make()
        pub.update(MAIN, jk_sample())
        sent = pub.publish(MAIN, jk_sample(), True, False, "stamp")
        self.assertGreater(sent, 10)
        self.assertEqual(factory.data("bms_main_pack_voltage"), [51.689])
        msg = factory.pubs["/xbot_monitoring/sensors/bms_main_pack_voltage/data"].msgs[0]
        self.assertEqual(msg.stamp, "stamp")

        self.assertEqual(pub.publish(MAIN, jk_sample(), True, True, "stamp"), 1)  # stale: status only
        self.assertEqual(factory.data("bms_main_pack_voltage"), [51.689])       # no new data
        self.assertEqual(factory.data("bms_main_pack_status"), ["OK", "Stale"])

    def test_unregistered_temperatures_are_skipped(self):
        pub, factory = self.make()
        pub.publish(MAIN, jk_sample(), True, False, "s")  # update() not called yet
        self.assertNotIn("/xbot_monitoring/sensors/bms_main_pack_temp_1/data", factory.pubs)

    def test_rate_limit(self):
        pub, _ = self.make(rate=2.0)
        self.assertTrue(pub.due(10.0))
        self.assertFalse(pub.due(10.3))
        self.assertTrue(pub.due(10.5))

    @staticmethod
    def loop(pub, period, n, jitter=0.002, seed=1):
        rnd = random.Random(seed)
        return sum(pub.due(100.0 + i * period + rnd.uniform(-jitter, jitter)) for i in range(n))

    def test_jitter_does_not_drop_cycles(self):
        # loop and sensor rate both 1 Hz: every iteration must publish, even if
        # it arrives a few ms early (previously ~1/3 of the cycles were dropped)
        pub, _ = self.make(rate=1.0)
        self.assertGreaterEqual(self.loop(pub, 1.0, 1000), 990)

    def test_still_throttles_faster_loop(self):
        # 10 Hz loop, 1 Hz sensor rate: about every tenth iteration publishes
        pub, _ = self.make(rate=1.0)
        published = self.loop(pub, 0.1, 1000)
        self.assertGreaterEqual(published, 99)
        self.assertLessEqual(published, 101)

    def test_no_burst_after_pause(self):
        pub, _ = self.make(rate=1.0)
        self.assertTrue(pub.due(0.0))
        self.assertTrue(pub.due(10.0))   # loop stalled for 10 s
        self.assertFalse(pub.due(10.1))  # no catch-up of the missed periods
        self.assertTrue(pub.due(11.0))


class ConfigTest(unittest.TestCase):
    def test_rate(self):
        self.assertEqual(parse_xbot_rate(1), 1.0)
        self.assertEqual(parse_xbot_rate("2.0"), 2.0)
        for bad in (0, -1, 2.5, "x", None):
            with self.assertRaises(ValueError):
                parse_xbot_rate(bad)

    def test_sensor_bms(self):
        devs = [MAIN, SECOND]
        self.assertEqual(parse_xbot_sensor_bms("", devs), [MAIN])       # primary
        self.assertEqual(parse_xbot_sensor_bms(None, devs), [MAIN])
        self.assertEqual(parse_xbot_sensor_bms("all", devs), devs)
        self.assertEqual(parse_xbot_sensor_bms("Second", devs), [SECOND])
        self.assertEqual(parse_xbot_sensor_bms("", [SECOND]), [])       # no primary -> disabled
        with self.assertRaises(ValueError):
            parse_xbot_sensor_bms("nope", devs)


class RealMessageTest(unittest.TestCase):
    def test_constants_match_xbot_msgs(self):
        try:
            from xbot_msgs.msg import SensorInfo
        except ImportError:
            self.skipTest("xbot_msgs not available")
        for name in dir(FakeSensorInfo):
            if name.isupper():
                self.assertEqual(getattr(FakeSensorInfo, name), getattr(SensorInfo, name), name)
        msg = xs.info_message(SensorInfo, xs.sensor_defs(MAIN, True)[0])
        msg.serialize(__import__("io").BytesIO())


JBD = DeviceConfig("pack_jbd", "jbd", "A5:C2:37:00:00:01", cell_count=4, nominal_capacity_ah=5.0, primary=True)


def jbd_sample(base=None, **changes):
    data = copy.deepcopy(base or fixtures.JBD_SAMPLE_4S)
    data.update(changes)
    return BmsSample.from_bridge("jbd_bms", data)


class JbdSensorTest(unittest.TestCase):
    def test_sensor_values(self):
        values = dict(xs.sensor_values(JBD, jbd_sample(), connected=True, stale=False))
        self.assertAlmostEqual(values["bms_pack_jbd_voltage"], 15.60)
        self.assertEqual(values["bms_pack_jbd_soc"], 100.0)
        self.assertEqual([values["bms_pack_jbd_temp_%d" % i] for i in (1, 2, 3)], [22.4, 22.3, 21.7])
        self.assertNotIn("bms_pack_jbd_temp_mosfet", values)
        self.assertEqual([values["bms_pack_jbd_cell_%02d" % i] for i in (1, 2, 3, 4)],
                         [3.909, 3.901, 3.895, 3.901])
        self.assertAlmostEqual(values["bms_pack_jbd_cell_delta"], 0.014)
        self.assertEqual(values["bms_pack_jbd_status"], "Full")

    def test_status_with_protection(self):
        s = jbd_sample(fixtures.JBD_SAMPLE_CELL_OVERVOLTAGE)
        self.assertEqual(xs.status_text(s), "Full, Cell overvoltage")
        self.assertEqual(xs.status_text(jbd_sample(current=-2.0, battery_level=50, problem_code=1 << 3 | 1 << 10)),
                         "Discharging, Pack undervoltage, Short circuit")

    def test_problem_flag_without_code_is_ok(self):
        self.assertEqual(xs.status_text(jbd_sample(battery_level=50, problem=True, problem_code=0)), "OK")
        self.assertEqual(xs.status_text(jk_sample(battery_level=50, problem=True, problem_code=0)), "OK")

    def test_late_registration_has_probes_but_no_mosfet_temperature(self):
        factory = Factory()
        pub = xs.XbotSensorPublisher([JBD], factory, FakeSensorInfo, FakeDouble, FakeString, 1.0)
        added = pub.update(JBD, jbd_sample())
        self.assertEqual(added, ["bms_pack_jbd_temp_1", "bms_pack_jbd_temp_2", "bms_pack_jbd_temp_3"])
        self.assertTrue(factory.pubs["/xbot_monitoring/sensors/bms_pack_jbd_temp_1/info"].latch)

    def test_ids_unique_with_jk_and_jbd(self):
        factory = Factory()
        pub = xs.XbotSensorPublisher([MAIN, JBD], factory, FakeSensorInfo, FakeDouble, FakeString, 1.0)
        pub.update(MAIN, jk_sample())
        pub.update(JBD, jbd_sample())
        self.assertEqual(len(pub.sensor_ids), len(set(pub.sensor_ids)))
        self.assertFalse(any(i.startswith("om_") for i in pub.sensor_ids))


if __name__ == "__main__":
    unittest.main()
