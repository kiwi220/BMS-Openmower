"""BridgeClient tests with a fake bridge process (runs on any Python 3)."""

import json
import os
import queue
import sys
import tempfile
import textwrap
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from bms_ble.bridge_client import BridgeClient  # noqa: E402

FAKE_BRIDGE = textwrap.dedent("""
    import json, sys
    cfg = json.loads(sys.stdin.readline())
    def emit(**kw):
        print(json.dumps(kw), flush=True)
    emit(event="ready", aiobmsble="fake", bleak="fake", devices=[d["name"] for d in cfg["devices"]])
    for d in cfg["devices"]:
        emit(event="sample", name=d["name"], bms_type="jikong_bms", data={"voltage": 25.0})
    print("not json", flush=True)
    print("some warning", file=sys.stderr, flush=True)
    if cfg.get("fake_mode") == "crash":
        sys.exit(3)
    for _ in sys.stdin:  # wait for EOF like the real bridge
        pass
    emit(event="log", level="info", msg="bye")
""")


class ListLogger:
    def __init__(self):
        self.records = []

    def __getattr__(self, level):
        return lambda msg, *a: self.records.append((level, msg % a if a else msg))


def collect(q, until, timeout=10.0):
    items = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not until(items):
        try:
            items.append(q.get(timeout=0.05))
        except queue.Empty:
            pass
    return items


class BridgeClientTest(unittest.TestCase):
    def setUp(self):
        fd, self.script = tempfile.mkstemp(suffix=".py")
        with os.fdopen(fd, "w") as f:
            f.write(FAKE_BRIDGE)

    def tearDown(self):
        os.unlink(self.script)

    def client(self, mode="", **kw):
        q = queue.Queue(maxsize=100)
        log = ListLogger()
        cfg = {"devices": [{"name": "main_pack", "type": "jk", "mac": "AA:BB:CC:DD:EE:FF"}], "fake_mode": mode}
        return BridgeClient([sys.executable, "-u", self.script], cfg, q, log, **kw), q, log

    def test_events_and_clean_stop(self):
        client, q, log = self.client()
        client.start()
        items = collect(q, lambda it: any(i["event"] == "sample" for i in it))
        self.assertEqual(items[0]["event"], "ready")
        sample = next(i for i in items if i["event"] == "sample")
        self.assertEqual(sample["data"], {"voltage": 25.0})
        self.assertIn("rx", sample)  # monotonic receive stamp added
        client.stop(timeout=5)
        self.assertFalse(client.running)
        rest = collect(q, lambda it: any(i["event"] == "log" for i in it), timeout=2)
        self.assertEqual([i["msg"] for i in rest if i["event"] == "log"], ["bye"])  # exited via stdin EOF
        self.assertFalse(any(i["event"] == "bridge_exit" for i in rest))
        levels = {lvl for lvl, _ in log.records}
        self.assertIn("error", levels)    # "not json"
        self.assertIn("warning", levels)  # stderr line

    def test_crash_is_reported_and_restarted(self):
        client, q, _ = self.client(mode="crash", restart_interval=0.1)
        client.start()
        items = collect(q, lambda it: sum(i["event"] == "ready" for i in it) >= 2)
        client.stop(timeout=5)
        exits = [i for i in items if i["event"] == "bridge_exit"]
        self.assertGreaterEqual(len(exits), 1)
        self.assertEqual(exits[0]["code"], 3)
        self.assertGreaterEqual(sum(i["event"] == "ready" for i in items), 2)

    def test_missing_interpreter(self):
        q = queue.Queue()
        log = ListLogger()
        client = BridgeClient(["/nonexistent/python3.12", self.script], {"devices": []}, q, log, restart_interval=0.1)
        client.start()
        items = collect(q, lambda it: len(it) >= 1)
        client.stop(timeout=2)
        self.assertEqual(items[0]["event"], "bridge_exit")
        self.assertTrue(any("setup_venv.sh" in msg for _, msg in log.records))

    def test_config_is_sent_with_protocol_version(self):
        client, q, _ = self.client()
        self.assertEqual(client._config["protocol"], 1)
        self.assertEqual(json.loads(json.dumps(client._config))["devices"][0]["name"], "main_pack")


if __name__ == "__main__":
    unittest.main()
