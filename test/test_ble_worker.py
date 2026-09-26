"""BleWorker tests against a fake bleak module (no Bluetooth hardware needed)."""

import asyncio
import os
import queue
import sys
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import fixtures  # noqa: E402
from jk_bms_ble import protocol  # noqa: E402
from jk_bms_ble.ble_worker import BleWorker, CellInfoEvent, ConnectionEvent, DeviceInfoEvent  # noqa: E402


class FakeChar:
    def __init__(self, handle, properties):
        self.uuid = protocol.CHARACTERISTIC_UUID
        self.handle = handle
        self.properties = properties


class FakeService:
    def __init__(self, characteristics):
        self.characteristics = characteristics


class FakeDevice:
    name = "JK-BD4A8S-4P"
    address = "C8:47:80:00:00:01"


class FakeBleak:
    """Configurable stand-in for the bleak module."""

    def __init__(self, device_found=True, drop_first_connection=True):
        self.device_found = device_found
        self.drop_first_connection = drop_first_connection
        self.connections = 0
        self.writes = []
        self.module = types.ModuleType("bleak")
        fake = self

        class BleakScanner:
            @staticmethod
            async def find_device_by_address(address, timeout=10.0):
                await asyncio.sleep(0.01)
                return FakeDevice() if fake.device_found else None

        class BleakClient:
            def __init__(self, device, disconnected_callback=None, timeout=10.0):
                self._disc_cb = disconnected_callback
                self._notify_cb = None
                self.is_connected = False
                # new BLE module layout: separate write (0x03) and notify (0x05) chars
                self.services = [FakeService([
                    FakeChar(3, ["write-without-response", "write"]),
                    FakeChar(5, ["notify"]),
                ])]

            async def __aenter__(self):
                fake.connections += 1
                self._index = fake.connections
                self.is_connected = True
                return self

            async def __aexit__(self, *exc):
                self.is_connected = False

            async def start_notify(self, char, callback):
                assert "notify" in char.properties
                self._notify_cb = callback

            async def stop_notify(self, char):
                pass

            async def write_gatt_char(self, char, data, response=False):
                assert "write-without-response" in char.properties
                fake.writes.append(bytes(data))
                loop = asyncio.get_event_loop()
                if data[4] == protocol.CMD_DEVICE_INFO:
                    self._send_chunked(loop, b"AT\r\n" + fixtures.DEVICE_INFO_JK02_32S_V11)
                elif data[4] == protocol.CMD_CELL_INFO:
                    self._send_chunked(loop, fixtures.CELL_INFO_JK02_32S_V11)
                    if fake.drop_first_connection and self._index == 1:
                        loop.call_later(0.2, self._drop)

            def _send_chunked(self, loop, data):
                for n, i in enumerate(range(0, len(data), 20)):
                    loop.call_later(0.001 * n, self._notify_cb, None, bytearray(data[i:i + 20]))

            def _drop(self):
                self.is_connected = False
                self._disc_cb(self)

        self.module.BleakScanner = BleakScanner
        self.module.BleakClient = BleakClient


def collect(q, until, timeout=5.0):
    events = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not until(events):
        try:
            events.append(q.get(timeout=0.05))
        except queue.Empty:
            pass
    return events


class BleWorkerTest(unittest.TestCase):
    def _run(self, fake, until, **kwargs):
        sys.modules["bleak"] = fake.module
        q = queue.Queue(maxsize=100)
        worker = BleWorker("C8:47:80:00:00:01", q, reconnect_interval=0.1, **kwargs)
        worker.start()
        try:
            events = collect(q, until)
        finally:
            worker.stop(timeout=3.0)
            del sys.modules["bleak"]
        self.assertFalse(worker.is_alive(), "worker thread did not stop")
        return events

    def test_connect_stream_disconnect_reconnect(self):
        fake = FakeBleak()

        def done(events):
            return sum(isinstance(e, CellInfoEvent) for e in events) >= 2

        events = self._run(fake, done)
        kinds = [type(e).__name__ + (":%s" % e.connected if isinstance(e, ConnectionEvent) else "")
                 for e in events]
        self.assertEqual(
            kinds[:6],
            ["ConnectionEvent:True", "DeviceInfoEvent", "CellInfoEvent",
             "ConnectionEvent:False", "ConnectionEvent:True", "DeviceInfoEvent"],
        )
        dev = next(e for e in events if isinstance(e, DeviceInfoEvent))
        self.assertEqual(dev.info.sw_version, "14.20")
        self.assertIs(dev.protocol, protocol.ProtocolVersion.JK02_32S)
        cell = next(e for e in events if isinstance(e, CellInfoEvent))
        self.assertAlmostEqual(cell.info.total_voltage, 51.689)
        self.assertEqual(fake.connections, 2)
        self.assertEqual(fake.writes[0], protocol.build_command(protocol.CMD_DEVICE_INFO))
        self.assertEqual(fake.writes[1], protocol.build_command(protocol.CMD_CELL_INFO))

    def test_device_not_found_keeps_retrying(self):
        fake = FakeBleak(device_found=False)
        start = time.monotonic()
        events = self._run(fake, lambda ev: time.monotonic() - start > 0.5)
        self.assertEqual(events, [])  # no connection events without a connection
        self.assertEqual(fake.connections, 0)

    def test_stall_triggers_rerequest_then_reconnect(self):
        fake = FakeBleak(drop_first_connection=False)
        orig = fake.module.BleakClient.write_gatt_char

        async def silent_after_first(self, char, data, response=False):
            if data[4] == protocol.CMD_CELL_INFO and any(w[4] == 0x96 for w in fake.writes):
                fake.writes.append(bytes(data))  # swallow: BMS stops streaming
                return
            await orig(self, char, data, response)

        fake.module.BleakClient.write_gatt_char = silent_after_first

        def reconnected(events):
            return [e.connected for e in events if isinstance(e, ConnectionEvent)][:3] == [True, False, True]

        events = self._run(fake, reconnected, data_timeout=0.2, max_request_retries=2)
        self.assertTrue(reconnected(events), events)
        rerequests = [w for w in fake.writes if w[4] == protocol.CMD_CELL_INFO]
        self.assertGreaterEqual(len(rerequests), 3)  # initial + 2 retries


if __name__ == "__main__":
    unittest.main()
