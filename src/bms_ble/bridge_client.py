"""Supervises the aiobmsble bridge process (no ROS imports).

A supervisor thread starts ``bridge/bms_bridge.py`` with the Python >= 3.12
interpreter, sends the JSON config on stdin, reads JSON lines from stdout and
puts them as dicts into a thread-safe ``queue.Queue``. If the bridge exits,
a ``{"event": "bridge_exit"}`` item is queued and the bridge is restarted.
Closing stdin on stop() makes the bridge shut down cleanly.
"""

import json
import logging
import queue
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional

PROTOCOL_VERSION = 1


class BridgeClient:
    def __init__(
        self,
        command: List[str],
        config: Dict[str, Any],
        out_queue: "queue.Queue",
        logger=None,
        restart_interval: float = 5.0,
    ) -> None:
        self._command = list(command)
        self._config = dict(config, protocol=PROTOCOL_VERSION)
        self._out = out_queue
        self._log = logger or logging.getLogger("bms_ble.bridge_client")
        self._restart_interval = restart_interval
        self._stopping = threading.Event()
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._supervise, name="bms_bridge_supervisor", daemon=True)

    # ------------------------------------------------------------------ API

    def start(self) -> None:
        self._thread.start()

    def stop(self, timeout: float = 10.0) -> None:
        self._stopping.set()
        with self._lock:
            proc = self._proc
        if proc is not None:
            self._close_stdin(proc)  # bridge disconnects all BMS and exits
            try:
                proc.wait(timeout)
            except subprocess.TimeoutExpired:
                self._log.warning("bridge did not exit within %.0f s, terminating", timeout)
                proc.terminate()
                try:
                    proc.wait(3.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self._thread.join(timeout)

    @property
    def running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    # ------------------------------------------------------------------ internals

    def _put(self, item: Dict[str, Any]) -> None:
        item.setdefault("rx", time.monotonic())
        while True:
            try:
                self._out.put_nowait(item)
                return
            except queue.Full:
                try:
                    self._out.get_nowait()  # drop oldest, never block the reader
                except queue.Empty:
                    pass

    @staticmethod
    def _close_stdin(proc: subprocess.Popen) -> None:
        try:
            if proc.stdin and not proc.stdin.closed:
                proc.stdin.close()
        except OSError:
            pass

    def _supervise(self) -> None:
        while not self._stopping.is_set():
            code = self._run_once()
            if self._stopping.is_set():
                break
            self._put({"event": "bridge_exit", "code": code})
            self._stopping.wait(self._restart_interval)

    def _run_once(self) -> Optional[int]:
        try:
            proc = subprocess.Popen(
                self._command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1,
            )
        except OSError as exc:
            self._log.error(
                "cannot start bridge %s: %s (run bridge/setup_venv.sh and set bridge_python)",
                " ".join(self._command), exc,
            )
            return None

        with self._lock:
            self._proc = proc
        stderr_thread = threading.Thread(
            target=self._pump_stderr, args=(proc,), name="bms_bridge_stderr", daemon=True
        )
        stderr_thread.start()
        try:
            proc.stdin.write(json.dumps(self._config) + "\n")
            proc.stdin.flush()
        except OSError as exc:
            self._log.error("cannot send config to bridge: %s", exc)
        if self._stopping.is_set():
            self._close_stdin(proc)

        for line in proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                item = json.loads(line)
            except ValueError:
                self._log.error("invalid line from bridge: %r", line[:200])
                continue
            if isinstance(item, dict) and "event" in item:
                self._put(item)
            else:
                self._log.error("unexpected message from bridge: %r", line[:200])

        code = proc.wait()
        stderr_thread.join(2.0)
        with self._lock:
            self._proc = None
        for pipe in (proc.stdin, proc.stdout, proc.stderr):
            try:
                pipe.close()
            except OSError:
                pass
        return code

    def _pump_stderr(self, proc: subprocess.Popen) -> None:
        for line in proc.stderr:
            line = line.rstrip()
            if line:
                self._log.warning("bridge stderr: %s", line)
