"""End-to-end tests of bridge/bms_bridge.py with the REAL aiobmsble package.

Only the BLE transport is faked: a fake BleakClient answers the JK commands
with real JK-BMS frames (test/fixtures.py). Requires Python >= 3.12 with
aiobmsble installed (bridge venv); skipped otherwise, e.g. under ROS' 3.8:

    ~/.local/share/bms_ble/venv/bin/python -m unittest test_bridge_aiobmsble
"""

import asyncio
import io
import json
import logging
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "bridge"))

try:
    if sys.version_info < (3, 12):
        raise ImportError("Python >= 3.12 required")
    import aiobmsble.basebms as basebms
    from bleak.backends.device import BLEDevice
    from bleak.backends.scanner import AdvertisementData

    import bms_bridge
    import fixtures
except ImportError as exc:  # pragma: no cover - depends on environment
    SKIP_REASON = str(exc)
else:
    SKIP_REASON = ""

JK_MAC = "C8:47:80:00:00:01"
FFE0 = "0000ffe0-0000-1000-8000-00805f9b34fb"
FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
FF00 = "0000ff00-0000-1000-8000-00805f9b34fb"
FF01 = "0000ff01-0000-1000-8000-00805f9b34fb"
FF02 = "0000ff02-0000-1000-8000-00805f9b34fb"
READY_MSG = bytes.fromhex("aa5590ebc80101" + "00" * 12 + "44")
JBD_MAC = "A5:C2:37:00:00:01"
JBD_PASSWORD_OK = bytes.fromhex("ffaa15010016")      # ff aa 15 | len 1 | 00 = accepted | crc
JBD_PASSWORD_REFUSED = bytes.fromhex("ffaa15010117")  # status 01 = wrong password


class FakeChar:
    def __init__(self, handle, properties, uuid=FFE1):
        self.uuid = uuid
        self.handle = handle
        self.properties = properties


class FakeServices:
    def __init__(self, protocol="jk"):
        if protocol == "jbd":  # JBD: write on ff02, notify on ff01
            self.chars = [FakeChar(0x0B, ["write-without-response", "write"], FF02), FakeChar(0x0E, ["notify"], FF01)]
        else:
            self.chars = [FakeChar(3, ["write-without-response", "write"]), FakeChar(5, ["notify"])]
        self.characteristics = self.chars

    def __iter__(self):
        return iter([self])

    def get_characteristic(self, spec):
        for c in self.chars:
            if spec in (c.handle, c.uuid) or (isinstance(spec, str) and c.uuid[4:8] == spec.lower()):
                return c
        return None

    def get_service(self, _uuid):
        return None


class FakeLink:
    """Shared state of the simulated BMS radio."""

    def __init__(self):
        self.connects = 0
        self.clients = []
        self.writes = []
        self.protocol = "jk"             # default protocol of the simulated radio
        self.protocol_by_mac = {}        # per device override, e.g. {JBD_MAC: "jbd"}
        self.jbd_basic = None            # JBD basic info frame to answer with (default: 4S fixture)
        self.jbd_password_reply = JBD_PASSWORD_OK


def make_fake_client(link):
    class FakeClient:
        def __init__(self, device, disconnected_callback=None, services=None, **_kw):
            self._disc_cb = disconnected_callback
            self._notify = None
            self.is_connected = False
            self.protocol = link.protocol_by_mac.get(device.address, link.protocol)
            self.services = FakeServices(self.protocol)
            link.clients.append(self)

        async def disconnect(self):
            self.is_connected = False
            return True

        async def start_notify(self, _char, callback):
            self._notify = callback

        async def stop_notify(self, _char):
            self._notify = None

        async def write_gatt_char(self, _char, data, response=None):
            link.writes.append(bytes(data))
            loop = asyncio.get_running_loop()
            if self.protocol == "jbd":
                if data.startswith(b"\xff\xaa"):  # password handshake
                    self._send(loop, link.jbd_password_reply)
                elif data[2] == 0x03:
                    self._send(loop, link.jbd_basic or fixtures.JBD_BASIC_INFO_4S)
                elif data[2] == 0x04:
                    self._send(loop, fixtures.JBD_CELL_INFO_4S)
                elif data[2] == 0x05:
                    self._send(loop, fixtures.JBD_HW_VERSION)
                return
            if data[4] == 0x97:
                self._send(loop, fixtures.DEVICE_INFO_JK02_32S_V11)
                self._send(loop, READY_MSG, delay=0.05)
            elif data[4] == 0x96:
                self._send(loop, fixtures.CELL_INFO_JK02_32S_V11)

        def _send(self, loop, data, delay=0.0):
            for n, i in enumerate(range(0, len(data), 20)):
                loop.call_later(delay + 0.001 * n, self._notify, None, bytearray(data[i:i + 20]))

        def drop(self):
            self.is_connected = False
            if self._disc_cb:
                self._disc_cb(self)

    async def establish_connection(client_class, device, name, disconnected_callback=None, **kw):
        link.connects += 1
        client = FakeClient(device, disconnected_callback)
        client.is_connected = True
        return client

    async def close_stale_connections(*_a, **_kw):
        return None

    return FakeClient, establish_connection, close_stale_connections


def jbd_adv(name="JBD-SP04S034-L4S-200A-B-U"):
    return AdvertisementData(
        local_name=name, manufacturer_data={}, service_data={},
        service_uuids=[FF00], tx_power=None, rssi=-60, platform_data=(),
    )


def jk_adv(name="JK-BD4A8S-4P"):
    return AdvertisementData(
        local_name=name, manufacturer_data={0x4B4A: b"\x00\x01"}, service_data={},
        service_uuids=[FFE0], tx_power=None, rssi=-60, platform_data=(),
    )


@unittest.skipIf(SKIP_REASON, SKIP_REASON)
class BridgeAiobmsbleTest(unittest.TestCase):
    def setUp(self):
        self.link = FakeLink()
        fake_client, establish, close_stale = make_fake_client(self.link)
        self._orig = (basebms.BleakClient, basebms.establish_connection, basebms.close_stale_connections)
        basebms.BleakClient = fake_client
        basebms.establish_connection = establish
        basebms.close_stale_connections = close_stale
        self.stream = io.StringIO()
        self._handler = bms_bridge.EmitLogHandler(bms_bridge.Emitter(self.stream))
        bms_bridge.log.addHandler(self._handler)

    def tearDown(self):
        bms_bridge.log.removeHandler(self._handler)
        basebms.BleakClient, basebms.establish_connection, basebms.close_stale_connections = self._orig

    def events(self, kind=None):
        out = [json.loads(line) for line in self.stream.getvalue().splitlines()]
        return [e for e in out if kind is None or e["event"] == kind]

    def run_bridge(self, dev_type, until, adv=None, timeout=15.0, on_tick=None, mac=JK_MAC,
                   device_name="JK-BD4A8S-4P", password=""):
        cfg = bms_bridge.BridgeConfig(
            devices=(bms_bridge.DeviceConfig("main_pack", dev_type, mac, password),),
            poll_interval=0.05, reconnect_interval=0.1, connect_timeout=10, update_timeout=5,
        )
        bridge = bms_bridge.Bridge(cfg, bms_bridge.Emitter(self.stream))

        async def fake_find(_dev):
            return BLEDevice(mac, device_name, None), adv or jk_adv()

        bridge.find_device = fake_find

        async def scenario():
            runner = asyncio.create_task(bridge.run())
            deadline = asyncio.get_running_loop().time() + timeout
            while not until(self) and asyncio.get_running_loop().time() < deadline:
                if on_tick:
                    on_tick(self)
                await asyncio.sleep(0.05)
            bridge.stop_event.set()
            await asyncio.wait_for(runner, 10)

        asyncio.run(scenario())

    def test_jk_sample_values_from_real_frames(self):
        self.run_bridge("jk", lambda t: len(t.events("sample")) >= 2)
        samples = self.events("sample")
        self.assertGreaterEqual(len(samples), 2)
        s = samples[0]
        self.assertEqual(s["bms_type"], "jikong_bms")
        d = s["data"]
        # expected values: esphome-jk-bms annotations of this frame
        self.assertAlmostEqual(d["voltage"], 51.689)
        self.assertAlmostEqual(d["current"], 0.0)
        self.assertEqual(d["battery_level"], 52)
        self.assertEqual(len(d["cell_voltages"]), 16)
        self.assertAlmostEqual(d["cell_voltages"][0], 3.246)
        self.assertAlmostEqual(d["cycle_charge"], 1.043)
        self.assertEqual(d["design_capacity"], 2)
        self.assertTrue(d["chrg_mosfet"])
        self.assertTrue(d["dischrg_mosfet"])
        self.assertEqual(d["problem_code"], 0)
        temps = {(t["type"], t["value"]) for t in d["temp_values"]}
        self.assertIn(("MOSFET", 17.3), temps)
        generic = [t["value"] for t in d["temp_values"] if t["type"] == "GENERIC"]
        # fw 14.20 frame: T1, T2 plus T4/T5 (fw >= 14); fw 11.x reports only T1, T2
        self.assertEqual(generic[:2], [17.7, 17.7])

        states = self.events("state")
        self.assertTrue(states[0]["connected"])
        info = self.events("device_info")
        self.assertEqual(info[0]["info"].get("sw_version"), "14.20")

    def test_auto_detects_jk_by_manufacturer_id(self):
        self.run_bridge("auto", lambda t: len(t.events("sample")) >= 1)
        self.assertEqual(self.events("sample")[0]["bms_type"], "jikong_bms")

    def test_link_loss_is_reported_and_reconnected(self):
        dropped = []

        def tick(t):
            if not dropped and t.events("sample"):
                dropped.append(True)
                t.link.clients[-1].drop()

        def done(t):
            return [e["connected"] for e in t.events("state")][:3] == [True, False, True]

        self.run_bridge("jk", done, on_tick=tick)
        self.assertEqual([e["connected"] for e in self.events("state")][:3], [True, False, True])
        self.assertGreaterEqual(self.link.connects, 2)
        warnings = [e["msg"] for e in self.events("log") if e["level"] == "warning"]
        self.assertTrue(any("connection lost" in w for w in warnings), warnings)


    def test_jk_password_is_not_sent(self):
        old_level = bms_bridge.log.level
        bms_bridge.log.setLevel(logging.INFO)  # main() does this in production
        self.addCleanup(bms_bridge.log.setLevel, old_level)
        self.run_bridge("jk", lambda t: len(t.events("sample")) >= 1, password="1234")
        self.assertTrue(self.events("sample"))
        self.assertFalse(any(b"1234" in w for w in self.link.writes))
        infos = [e["msg"] for e in self.events("log") if e["level"] == "info"]
        self.assertTrue(any("ble_connect_password ignored" in m for m in infos), infos)


@unittest.skipIf(SKIP_REASON, SKIP_REASON)
class JbdBridgeTest(BridgeAiobmsbleTest):
    """Same bridge, JBD protocol (service ff00, real frames from esphome-jbd-bms)."""

    def setUp(self):
        super().setUp()
        self.link.protocol = "jbd"

    def run_jbd(self, until, **kw):
        self.run_bridge("jbd", until, adv=jbd_adv(), mac=JBD_MAC, device_name="JBD-SP04S034-L4S-200A-B-U", **kw)

    def test_jbd_sample_values_from_real_frames(self):
        self.run_jbd(lambda t: len(t.events("sample")) >= 2)
        samples = self.events("sample")
        self.assertGreaterEqual(len(samples), 2)
        self.assertEqual(samples[0]["bms_type"], "jbd_bms")
        d = samples[0]["data"]
        # expected values: esphome-jbd-bms annotations of these frames
        self.assertAlmostEqual(d["voltage"], 15.60)
        self.assertAlmostEqual(d["current"], 0.0)
        self.assertEqual(d["battery_level"], 100)
        self.assertAlmostEqual(d["cycle_charge"], 4.98)
        self.assertEqual(d["design_capacity"], 5)
        self.assertEqual(d["cycles"], 0)
        self.assertEqual(d["cell_voltages"], [3.909, 3.901, 3.895, 3.901])
        self.assertTrue(d["chrg_mosfet"])
        self.assertTrue(d["dischrg_mosfet"])
        self.assertEqual(d["problem_code"], 0)
        # NTCs are typed CELL by aiobmsble (no MOSFET temperature)
        self.assertEqual([(t["type"], t["value"]) for t in d["temp_values"]],
                         [("CELL", 22.4), ("CELL", 22.3), ("CELL", 21.7)])
        info = self.events("device_info")[0]["info"]
        self.assertEqual(info["sw_version"], "8.0")
        self.assertEqual(info["hw_version"], "JBD-SP04S034-L4S-200A-B-U")
        self.assertTrue(self.events("state")[0]["connected"])

    def test_jbd_cell_overvoltage_protection_capture(self):
        self.link.jbd_basic = fixtures.JBD_BASIC_INFO_CELL_OVERVOLTAGE
        self.run_jbd(lambda t: len(t.events("sample")) >= 1)
        d = self.events("sample")[0]["data"]
        self.assertEqual(d["problem_code"], 0x0001)   # cell overvoltage protection
        self.assertTrue(d["problem"])
        self.assertAlmostEqual(d["voltage"], 14.28)
        self.assertEqual(d["design_capacity"], 280)
        self.assertEqual(d["cycles"], 8)
        self.assertFalse(d["chrg_mosfet"])
        self.assertTrue(d["dischrg_mosfet"])

    def test_jbd_password_is_sent_and_accepted(self):
        self.run_jbd(lambda t: len(t.events("sample")) >= 1, password="123456")
        init = [w for w in self.link.writes if w.startswith(b"\xff\xaa\x15")]
        self.assertEqual(len(init), 1)
        self.assertEqual(init[0][3], 6)
        self.assertEqual(init[0][4:10], b"123456")
        self.assertTrue(self.events("sample"))

    def test_jbd_wrong_password_reports_failure_without_samples(self):
        self.link.jbd_password_reply = JBD_PASSWORD_REFUSED
        warned = lambda t: any("connect failed" in e["msg"] for e in t.events("log"))  # noqa: E731
        self.run_jbd(warned, password="000000")
        self.assertEqual(self.events("sample"), [])
        self.assertEqual(self.events("state"), [])  # never reported as connected
        msgs = [e["msg"] for e in self.events("log") if e["level"] == "warning"]
        self.assertTrue(any("PermissionError" in m for m in msgs), msgs)

    def test_jbd_link_loss_is_reported_and_reconnected(self):
        dropped = []

        def tick(t):
            if not dropped and t.events("sample"):
                dropped.append(True)
                t.link.clients[-1].drop()

        def done(t):
            return [e["connected"] for e in t.events("state")][:3] == [True, False, True]

        self.run_jbd(done, on_tick=tick)
        self.assertEqual([e["connected"] for e in self.events("state")][:3], [True, False, True])

    # inherited JK tests are not re-run here
    test_jk_sample_values_from_real_frames = None
    test_auto_detects_jk_by_manufacturer_id = None
    test_link_loss_is_reported_and_reconnected = None
    test_jk_password_is_not_sent = None


@unittest.skipIf(SKIP_REASON, SKIP_REASON)
class MixedBmsTest(BridgeAiobmsbleTest):
    """One JK and one JBD in the same bridge, each with its own protocol."""

    def test_jk_and_jbd_side_by_side(self):
        self.link.protocol_by_mac = {JBD_MAC: "jbd"}
        cfg = bms_bridge.BridgeConfig(
            devices=(
                bms_bridge.DeviceConfig("pack_jk", "jk", JK_MAC),
                bms_bridge.DeviceConfig("pack_jbd", "jbd", JBD_MAC),
            ),
            poll_interval=0.05, reconnect_interval=0.1,
        )
        bridge = bms_bridge.Bridge(cfg, bms_bridge.Emitter(self.stream))

        async def fake_find(dev):
            return (BLEDevice(dev.mac, dev.name, None), jk_adv() if dev.type == "jk" else jbd_adv())

        bridge.find_device = fake_find

        async def scenario():
            runner = asyncio.create_task(bridge.run())
            deadline = asyncio.get_running_loop().time() + 15
            while asyncio.get_running_loop().time() < deadline:
                got = {e["name"] for e in self.events("sample")}
                if got == {"pack_jk", "pack_jbd"}:
                    break
                await asyncio.sleep(0.05)
            bridge.stop_event.set()
            await asyncio.wait_for(runner, 10)

        asyncio.run(scenario())
        by_name = {}
        for e in self.events("sample"):
            by_name.setdefault(e["name"], e)
        self.assertEqual(by_name["pack_jk"]["bms_type"], "jikong_bms")
        self.assertAlmostEqual(by_name["pack_jk"]["data"]["voltage"], 51.689)
        self.assertEqual(by_name["pack_jbd"]["bms_type"], "jbd_bms")
        self.assertAlmostEqual(by_name["pack_jbd"]["data"]["voltage"], 15.60)
        connected = {e["name"] for e in self.events("state") if e["connected"]}
        self.assertEqual(connected, {"pack_jk", "pack_jbd"})

    # inherited tests are not re-run here
    test_jk_sample_values_from_real_frames = None
    test_auto_detects_jk_by_manufacturer_id = None
    test_link_loss_is_reported_and_reconnected = None
    test_jk_password_is_not_sent = None


@unittest.skipIf(SKIP_REASON, SKIP_REASON)
class ConnectionLimitTest(BridgeAiobmsbleTest):
    def test_second_device_waits_for_free_slot(self):
        cfg = bms_bridge.BridgeConfig(
            devices=(
                bms_bridge.DeviceConfig("pack_a", "jk", JK_MAC),
                bms_bridge.DeviceConfig("pack_b", "jk", "C8:47:80:00:00:02"),
            ),
            poll_interval=0.05, reconnect_interval=0.1, max_connections=1,
        )
        bridge = bms_bridge.Bridge(cfg, bms_bridge.Emitter(self.stream))

        async def fake_find(dev):
            return BLEDevice(dev.mac, "JK-BD4A8S-4P", None), jk_adv()

        bridge.find_device = fake_find

        async def scenario():
            runner = asyncio.create_task(bridge.run())
            await asyncio.sleep(1.0)
            bridge.stop_event.set()
            await asyncio.wait_for(runner, 10)

        asyncio.run(scenario())
        names = {e["name"] for e in self.events("sample")}
        self.assertEqual(len(names), 1)  # only one pack gets the single slot
        warnings = [e["msg"] for e in self.events("log") if e["level"] == "warning"]
        self.assertTrue(any("connection limit reached (max_connections=1" in w for w in warnings), warnings)

    # inherited tests are not re-run here
    test_jk_sample_values_from_real_frames = None
    test_auto_detects_jk_by_manufacturer_id = None
    test_link_loss_is_reported_and_reconnected = None
    test_jk_password_is_not_sent = None


@unittest.skipIf(SKIP_REASON, SKIP_REASON)
class ResolveTypeTest(unittest.TestCase):
    def resolve(self, dev_type, name, services=None):
        bridge = bms_bridge.Bridge(
            bms_bridge.BridgeConfig(devices=(bms_bridge.DeviceConfig("p", dev_type, JK_MAC),)),
            bms_bridge.Emitter(io.StringIO()),
        )
        adv = AdvertisementData(
            local_name=name, manufacturer_data={}, service_data={},
            service_uuids=services or [FFE0, "0000fee7-0000-1000-8000-00805f9b34fb"],
            tx_power=None, rssi=-60, platform_data=(),
        )

        async def go():
            return await bridge.resolve_class(bridge.cfg.devices[0], BLEDevice(JK_MAC, name, None), adv)

        with self.assertLogs("bms_bridge", level="INFO") as logs:
            cls = asyncio.run(go())
            bms_bridge.log.info("end")  # assertLogs needs at least one record
        return (bms_bridge.module_name(cls) if cls else None), logs.output

    def test_ant_legacy_name(self):
        self.assertEqual(self.resolve("ant", "ANT-BLE16ZMUB")[0], "ant_leg_bms")

    def test_ant_new_name(self):
        self.assertEqual(self.resolve("ant", "ANT-BLE24AHA")[0], "ant_bms")

    def test_ant_unknown_variant_falls_back_with_warning(self):
        module, logs = self.resolve("ant", "ANT-BLEUB1234")
        self.assertEqual(module, "ant_bms")
        self.assertTrue(any("matches no aiobmsble ANT pattern" in line for line in logs), logs)

    def test_auto_unknown_ant_name(self):
        self.assertEqual(self.resolve("auto", "ANT-BLEUB1234")[0], "ant_bms")

    def test_auto_unknown_device(self):
        module, logs = self.resolve("auto", "SomethingElse")
        self.assertIsNone(module)
        self.assertTrue(any("not recognized" in line for line in logs), logs)

    def test_jbd_explicit_type_ignores_the_name(self):
        for name in ("JBD-SP04S034", "xiaoxiang BMS", "whatever"):
            self.assertEqual(self.resolve("jbd", name, [FF00])[0], "jbd_bms")

    def test_jbd_auto_detection_by_name(self):
        self.assertEqual(self.resolve("auto", "JBD-SP04S034-L4S-200A-B-U", [FF00])[0], "jbd_bms")

    def test_auto_unknown_name_with_jbd_service_gives_hint(self):
        module, logs = self.resolve("auto", "xiaoxiang BMS", [FF00])
        self.assertIsNone(module)
        self.assertTrue(any("try type: jbd" in line for line in logs), logs)

    def test_auto_unknown_device_without_jbd_service_has_no_hint(self):
        _, logs = self.resolve("auto", "SomethingElse")
        self.assertFalse(any("try type: jbd" in line for line in logs), logs)

    def test_explicit_types(self):
        self.assertEqual(self.resolve("jk", "whatever")[0], "jikong_bms")
        self.assertEqual(self.resolve("ant_leg", "ANT-BLEUB1")[0], "ant_leg_bms")


@unittest.skipIf(SKIP_REASON, SKIP_REASON)
class EmitterTest(unittest.TestCase):
    def test_serializes_library_types(self):
        from aiobmsble import BMSMode, TempSensor

        stream = io.StringIO()
        bms_bridge.Emitter(stream).emit(
            "sample", data={"temp_values": [TempSensor(21.5, TempSensor.T.MOSFET)], "battery_mode": BMSMode.FLOAT}
        )
        ev = json.loads(stream.getvalue())
        self.assertEqual(ev["data"]["temp_values"], [{"value": 21.5, "type": "MOSFET"}])
        self.assertEqual(ev["data"]["battery_mode"], "FLOAT")


if __name__ == "__main__":
    unittest.main()
