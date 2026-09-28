"""bms_list validation tests."""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from bms_ble.config import parse_bms_list, parse_pack_connection  # noqa: E402


def entry(**kw):
    base = {"name": "main_pack", "type": "jk", "mac": "aa:bb:cc:dd:ee:ff", "cell_count": 7, "nominal_capacity_ah": 10.0}
    base.update(kw)
    return base


class ConfigTest(unittest.TestCase):
    def test_single_becomes_primary(self):
        (d,) = parse_bms_list([entry()])
        self.assertTrue(d.primary)
        self.assertEqual(d.mac, "AA:BB:CC:DD:EE:FF")

    def test_two_devices(self):
        devs = parse_bms_list([
            entry(primary=True),
            entry(name="second", type="ant", mac="11:22:33:44:55:66", ble_connect_password="1234"),
        ])
        self.assertEqual([d.primary for d in devs], [True, False])
        self.assertEqual(devs[1].password, "1234")
        self.assertEqual(devs[1].bridge_dict(), {"name": "second", "type": "ant", "mac": "11:22:33:44:55:66",
                                                  "password": "1234"})

    def test_no_primary_with_several_is_allowed(self):
        devs = parse_bms_list([entry(), entry(name="b", mac="11:22:33:44:55:66")])
        self.assertFalse(any(d.primary for d in devs))

    def test_errors(self):
        cases = [
            [],
            [entry(name="bad name")],
            [entry(name="combined")],
            [entry(type="daly")],
            [entry(mac="nope")],
            [entry(cell_count=0)],
            [entry(nominal_capacity_ah=-1)],
            [entry(), entry()],                                           # duplicate name
            [entry(), entry(name="b")],                                   # duplicate MAC
            [entry(primary=True), entry(name="b", mac="11:22:33:44:55:66", primary=True)],
        ]
        for raw in cases:
            with self.assertRaises(ValueError, msg=raw):
                parse_bms_list(raw)

    def test_module_name_type(self):
        self.assertEqual(parse_bms_list([entry(type="jikong_bms")])[0].type, "jikong_bms")

    def test_legacy_single_parameters(self):
        (d,) = parse_bms_list([], {"bms_mac_address": "C8:47:80:00:00:01", "cell_count": 7})
        self.assertEqual((d.name, d.type, d.primary), ("main", "jk", True))

    def test_pack_connection(self):
        self.assertEqual(parse_pack_connection(None), "")
        self.assertEqual(parse_pack_connection("Series"), "series")
        with self.assertRaises(ValueError):
            parse_pack_connection("mixed")


if __name__ == "__main__":
    unittest.main()
