"""Protocol tests against real frames (expected values from esphome-jk-bms)."""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import fixtures  # noqa: E402
from jk_bms_ble import protocol  # noqa: E402
from jk_bms_ble.protocol import ProtocolVersion  # noqa: E402


class CommandTest(unittest.TestCase):
    def test_cell_info_command(self):
        cmd = protocol.build_command(protocol.CMD_CELL_INFO)
        self.assertEqual(len(cmd), 20)
        self.assertEqual(cmd, bytes.fromhex("aa5590eb96" + "00" * 14 + "10"))

    def test_device_info_command(self):
        cmd = protocol.build_command(protocol.CMD_DEVICE_INFO)
        self.assertEqual(cmd[:5], bytes.fromhex("aa5590eb97"))
        self.assertEqual(cmd[-1], 0x11)

    def test_checksum_matches_known_ready_message(self):
        # "BMS ready" frame from aiobmsble: header c8 01 01 + 12 x 00 + 0x44
        self.assertEqual(protocol.build_command(0xC8, b"\x01"), bytes.fromhex("aa5590ebc80101" + "00" * 12 + "44"))

    def test_value_too_long(self):
        with self.assertRaises(ValueError):
            protocol.build_command(0x96, bytes(14))


class AssemblerTest(unittest.TestCase):
    def _chunks(self, data, size):
        return [data[i:i + size] for i in range(0, len(data), size)]

    def test_reassembles_mtu_chunks(self):
        asm = protocol.FrameAssembler()
        out = []
        for chunk in self._chunks(fixtures.CELL_INFO_JK02_32S_V11, 20):
            out += asm.feed(chunk)
        self.assertEqual(len(out), 1)
        self.assertTrue(out[0].checksum_ok)
        self.assertEqual(out[0].frame_type, protocol.FRAME_TYPE_CELL_INFO)
        self.assertEqual(out[0].data, fixtures.CELL_INFO_JK02_32S_V11)

    def test_filters_at_message_and_garbage(self):
        asm = protocol.FrameAssembler()
        self.assertEqual(asm.feed(b"AT\r\n"), [])
        self.assertEqual(asm.feed(b"\x01\x02garbage"), [])
        out = []
        for chunk in self._chunks(fixtures.DEVICE_INFO_JK02_32S_V11, 128):
            out += asm.feed(chunk)
        self.assertEqual([f.frame_type for f in out], [protocol.FRAME_TYPE_DEVICE_INFO])

    def test_oversized_frame_with_trailer(self):
        # v19+ appends the 20 byte ready message; checksum is still at 299
        ready = protocol.build_command(0xC8, b"\x01")
        asm = protocol.FrameAssembler()
        out = asm.feed(fixtures.CELL_INFO_JK02_32S_V11 + ready)
        self.assertEqual(len(out), 1)
        self.assertTrue(out[0].checksum_ok)

    def test_two_frames_back_to_back(self):
        asm = protocol.FrameAssembler()
        data = fixtures.DEVICE_INFO_JK02_32S_V11 + fixtures.CELL_INFO_JK02_32S_V11
        out = []
        for chunk in self._chunks(data, 182):  # chunk boundaries cross frames
            out += asm.feed(chunk)
        self.assertEqual([f.frame_type for f in out], [0x03, 0x02])
        self.assertTrue(all(f.checksum_ok for f in out))

    def test_bad_checksum_reported(self):
        bad = bytearray(fixtures.CELL_INFO_JK02_32S_V11)
        bad[150] ^= 0xFF
        out = protocol.FrameAssembler().feed(bytes(bad))
        self.assertEqual(len(out), 1)
        self.assertFalse(out[0].checksum_ok)

    def test_truncated_frame_is_dropped_on_new_header(self):
        asm = protocol.FrameAssembler()
        self.assertEqual(asm.feed(fixtures.CELL_INFO_JK02_32S_V11[:200]), [])
        out = asm.feed(fixtures.CELL_INFO_JK02_32S_V11)
        self.assertEqual(len(out), 1)
        self.assertTrue(out[0].checksum_ok)


class DeviceInfoTest(unittest.TestCase):
    def test_v11_device_info(self):
        info = protocol.parse_device_info(fixtures.DEVICE_INFO_JK02_32S_V11)
        self.assertEqual(info.model, "JK_PB2A16S15P")
        self.assertEqual(info.hw_version, "14.XA")
        self.assertEqual(info.sw_version, "14.20")
        self.assertIs(info.protocol, ProtocolVersion.JK02_32S)

    def test_v10_device_info_selects_24s(self):
        info = protocol.parse_device_info(fixtures.DEVICE_INFO_JK02_24S_V10)
        self.assertEqual(info.sw_version, "10.07")
        self.assertIs(info.protocol, ProtocolVersion.JK02_24S)

    def test_sw_11_24_is_32s(self):
        # the user's JK-BD4A8S4P: hw 11.xw, sw 11.24
        self.assertIs(ProtocolVersion.from_sw_version("11.24"), ProtocolVersion.JK02_32S)


class CellInfo32STest(unittest.TestCase):
    def test_v11_frame(self):
        # esphome: 16 cells, 51.689 V, 0 A, T1 17.7, T2 17.7, MOS 17.3,
        # SOC 52 %, SOH 100 %, cap_rem 1.043 Ah, nom_cap 2.000 Ah, chg on, dsg on
        info = protocol.parse_cell_info(fixtures.CELL_INFO_JK02_32S_V11, ProtocolVersion.JK02_32S)
        self.assertEqual(info.enabled_cell_count, 16)
        self.assertAlmostEqual(info.cell_voltages[0], 3.246)
        self.assertAlmostEqual(min(info.cell_voltages), 3.216)
        self.assertAlmostEqual(info.total_voltage, 51.689)
        self.assertAlmostEqual(info.current, 0.0)
        self.assertAlmostEqual(info.mosfet_temperature, 17.3)
        self.assertEqual(info.temperatures, [17.7, 17.7])
        self.assertEqual(info.state_of_charge, 52)
        self.assertEqual(info.state_of_health, 100)
        self.assertAlmostEqual(info.remaining_capacity, 1.043)
        self.assertAlmostEqual(info.full_charge_capacity, 2.0)
        self.assertEqual(info.cycle_count, 0)
        self.assertTrue(info.charge_mosfet)
        self.assertTrue(info.discharge_mosfet)
        self.assertEqual(info.error_bitmask, 0)

    def test_v15_charging_frame(self):
        # esphome: 53.224 V, +31.881 A, T1 13.4, T2 12.8, MOS 12.9, SOC 25 %,
        # cap_rem 49.286 Ah, nom_cap 200 Ah, 9 cycles
        info = protocol.parse_cell_info(fixtures.CELL_INFO_JK02_32S_V15, ProtocolVersion.JK02_32S)
        self.assertEqual(info.enabled_cell_count, 16)
        self.assertAlmostEqual(info.total_voltage, 53.224)
        self.assertAlmostEqual(info.current, 31.881)
        self.assertAlmostEqual(info.mosfet_temperature, 12.9)
        self.assertEqual(info.temperatures, [13.4, 12.8])
        self.assertEqual(info.state_of_charge, 25)
        self.assertAlmostEqual(info.remaining_capacity, 49.286)
        self.assertAlmostEqual(info.full_charge_capacity, 200.0)
        self.assertEqual(info.cycle_count, 9)

    def _patched(self, **changes):
        frame = bytearray(fixtures.CELL_INFO_JK02_32S_V11)
        for pos, raw in changes.values():
            frame[pos:pos + len(raw)] = raw
        frame[299] = protocol.checksum(frame[:299])
        return bytes(frame)

    def test_negative_current_is_discharge(self):
        frame = self._patched(current=(158, (-2500).to_bytes(4, "little", signed=True)))
        info = protocol.parse_cell_info(frame)
        self.assertAlmostEqual(info.current, -2.5)

    def test_unplugged_ntc_is_none(self):
        # "NA" in the app: raw -2000, or sensor bit cleared in the present mask
        frame = self._patched(t2=(164, (-2000).to_bytes(2, "little", signed=True)))
        self.assertEqual(protocol.parse_cell_info(frame).temperatures, [17.7, None])
        frame = self._patched(mask=(214, (0b001).to_bytes(2, "little")))
        info = protocol.parse_cell_info(frame)
        self.assertEqual(info.temperatures, [None, None])
        self.assertAlmostEqual(info.mosfet_temperature, 17.3)

    def test_seven_cells_and_mosfets_off(self):
        frame = self._patched(
            mask=(70, (0x7F).to_bytes(4, "little")),
            chg=(198, b"\x00"),
            dsg=(199, b"\x00"),
            err=(166, (1 << 5 | 1 << 12).to_bytes(4, "little")),
        )
        info = protocol.parse_cell_info(frame)
        self.assertEqual(info.enabled_cell_count, 7)
        self.assertEqual(len(info.cell_voltages), 7)
        self.assertFalse(info.charge_mosfet)
        self.assertFalse(info.discharge_mosfet)
        self.assertEqual(info.errors, ["Battery pack overvoltage", "Battery pack undervoltage"])

    def test_wrong_frame_type(self):
        with self.assertRaises(protocol.ProtocolError):
            protocol.parse_cell_info(fixtures.DEVICE_INFO_JK02_32S_V11)


class CellInfo24STest(unittest.TestCase):
    def test_v10_frame(self):
        # esphome: 24 cells, 78.735 V, T1 20.7, T2 20.7, MOS 23.4, SOC 85 %,
        # cap_rem 42.804 Ah, nom_cap 50 Ah, chg on, dsg off
        info = protocol.parse_cell_info(fixtures.CELL_INFO_JK02_24S_V10, ProtocolVersion.JK02_24S)
        self.assertEqual(info.enabled_cell_count, 24)
        self.assertAlmostEqual(info.total_voltage, 78.735)
        self.assertAlmostEqual(info.mosfet_temperature, 23.4)
        self.assertEqual(info.temperatures, [20.7, 20.7])
        self.assertEqual(info.state_of_charge, 85)
        self.assertAlmostEqual(info.remaining_capacity, 42.804)
        self.assertAlmostEqual(info.full_charge_capacity, 50.0)
        self.assertTrue(info.charge_mosfet)
        self.assertFalse(info.discharge_mosfet)


if __name__ == "__main__":
    unittest.main()
