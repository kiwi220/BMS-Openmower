#!/usr/bin/env python3
"""aiobmsble bridge: reads BLE battery management systems, emits JSON lines.

Runs in its own Python >= 3.12 environment (aiobmsble does not support the
Python 3.8 of ROS Noetic). It is started by the ROS node, which keeps it
alive, and talks to it like this:

    stdin   first line: JSON config, then kept open. EOF -> clean shutdown
            (so the bridge never outlives the node).
    stdout  one JSON object per line ("event": ready | state | device_info |
            sample | log)
    stderr  unexpected output only (tracebacks etc.)

All BLE handling, protocol parsing and connection cleanup is done by
aiobmsble (patman15/BMS_BLE-HA). This file only adds scheduling: one task per
BMS, reconnect loop, hard timeouts, serialized scans and a connection limit.
"""

import asyncio
import contextlib
import enum
import importlib.metadata
import json
import logging
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

from bleak import BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

from aiobmsble import BMSConfig
from aiobmsble.basebms import BaseBMS
from aiobmsble.utils import bms_cls, bms_identify, bms_supported

PROTOCOL_VERSION = 1

# config "type" -> aiobmsble module; "ant" and "auto" are resolved at runtime
TYPE_MODULES = {"jk": "jikong_bms", "ant_leg": "ant_leg_bms", "ant_new": "ant_bms"}

log = logging.getLogger("bms_bridge")


def module_name(cls: type[BaseBMS]) -> str:
    """Short aiobmsble module name, e.g. 'jikong_bms' (get_bms_module() is dotted)."""
    return cls.get_bms_module().rsplit(".", 1)[-1]


# --------------------------------------------------------------------------- output


class Emitter:
    """Thread-safe JSON line writer (stdout is the only channel to the node)."""

    def __init__(self, stream=None) -> None:
        self._stream = stream or sys.stdout
        self._lock = threading.Lock()

    def emit(self, event: str, **fields: Any) -> None:
        line = json.dumps(_jsonable({"event": event, "t": time.time(), **fields}))
        with self._lock:
            try:
                self._stream.write(line + "\n")
                self._stream.flush()
            except (BrokenPipeError, ValueError):
                pass  # node is gone; stdin EOF will stop us


def _jsonable(obj: Any) -> Any:
    """Convert aiobmsble values (TempSensor, IntEnum, bytes, ...) to plain JSON types."""
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, enum.Enum):  # before int: BMSMode is an IntEnum
        return obj.name
    if isinstance(obj, (bool, int, float, str)) or obj is None:
        return obj
    if isinstance(obj, (bytes, bytearray)):
        return obj.hex()
    if hasattr(obj, "value") and hasattr(obj, "type"):  # aiobmsble.TempSensor
        return {"value": float(obj.value), "type": obj.type.name}
    return str(obj)


class EmitLogHandler(logging.Handler):
    """Forward Python logging (bridge + aiobmsble) to the node as log events."""

    def __init__(self, emitter: Emitter) -> None:
        super().__init__()
        self._emitter = emitter

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = record.getMessage()
        except Exception:  # malformed log call in a library
            msg = str(record.msg)
        self._emitter.emit("log", level=record.levelname.lower(), logger=record.name, msg=msg)


# --------------------------------------------------------------------------- config


@dataclass(frozen=True)
class DeviceConfig:
    name: str
    type: str
    mac: str
    password: str = ""


@dataclass(frozen=True)
class BridgeConfig:
    devices: tuple[DeviceConfig, ...]
    poll_interval: float = 1.0
    reconnect_interval: float = 5.0
    scan_timeout: float = 10.0
    connect_timeout: float = 45.0
    update_timeout: float = 20.0
    disconnect_timeout: float = 10.0
    max_connections: int = 4
    log_level: str = "WARNING"

    @staticmethod
    def from_dict(raw: dict[str, Any]) -> "BridgeConfig":
        devices = tuple(
            DeviceConfig(
                name=str(d["name"]),
                type=str(d.get("type", "auto")),
                mac=str(d["mac"]).upper(),
                password=str(d.get("password", "") or ""),
            )
            for d in raw.get("devices", [])
        )
        if not devices:
            raise ValueError("no devices configured")
        known = {f for f in BridgeConfig.__dataclass_fields__ if f != "devices"}
        options = {k: v for k, v in raw.items() if k in known}
        return BridgeConfig(devices=devices, **options)


# --------------------------------------------------------------------------- bridge


class Bridge:
    def __init__(self, cfg: BridgeConfig, emitter: Emitter) -> None:
        self.cfg = cfg
        self.out = emitter
        self.stop_event = asyncio.Event()
        self._scan_lock = asyncio.Lock()  # BlueZ rejects concurrent discovery sessions
        self._slots = asyncio.Semaphore(cfg.max_connections)
        self._connected: set[str] = set()

    # ---- helpers

    async def _sleep(self, seconds: float) -> None:
        with contextlib.suppress(TimeoutError):
            async with asyncio.timeout(seconds):
                await self.stop_event.wait()

    async def find_device(self, dev: DeviceConfig) -> tuple[BLEDevice, AdvertisementData] | None:
        """Scan until the configured MAC shows up; returns device + advertisement."""
        found: dict[str, AdvertisementData] = {}

        def match(device: BLEDevice, adv: AdvertisementData) -> bool:
            if device.address.upper() == dev.mac:
                found["adv"] = adv
                return True
            return False

        async with self._scan_lock:
            device = await BleakScanner.find_device_by_filter(match, timeout=self.cfg.scan_timeout)
        if device is None:
            return None
        return device, found["adv"]

    async def resolve_class(
        self, dev: DeviceConfig, device: BLEDevice, adv: AdvertisementData
    ) -> type[BaseBMS] | None:
        """Map the configured type to an aiobmsble BMS class."""
        name = adv.local_name or device.name or ""
        kind = dev.type

        if kind == "auto":
            cls = await bms_identify(adv, device.address)
            if cls is not None:
                log.info("%s: '%s' identified as %s", dev.name, name, module_name(cls))
                return cls
            if not name.upper().startswith("ANT"):
                log.warning(
                    "%s: device '%s' (%s) not recognized by aiobmsble; set type: jk or ant explicitly",
                    dev.name, name, dev.mac,
                )
                return None
            log.warning("%s: '%s' not recognized, name suggests ANT", dev.name, name)
            kind = "ant"

        if kind == "ant":
            leg = await bms_cls("ant_leg_bms")
            new = await bms_cls("ant_bms")
            for cls in (leg, new):
                if cls is not None and bms_supported(cls, adv, device.address):
                    return cls
            patterns = [p.get("local_name") for c in (leg, new) if c for p in c.matcher_dict_list()]
            log.warning(
                "%s: name '%s' matches no aiobmsble ANT pattern %s; trying ant_bms "
                "(set type: ant_leg if no data arrives)",
                dev.name, name, patterns,
            )
            return new

        module = TYPE_MODULES.get(kind, kind)  # also accepts raw aiobmsble module names
        cls = await bms_cls(module)
        if cls is None:
            log.error("%s: unknown BMS type/module '%s'", dev.name, kind)
        return cls

    async def _safe_disconnect(self, bms: BaseBMS) -> None:
        """Disconnect and close stale BlueZ connections, bounded in time."""
        try:
            async with asyncio.timeout(self.cfg.disconnect_timeout):
                await bms.disconnect(reset=True)
        except Exception as exc:  # BlueZ can hang or fail here; never block the loop
            log.warning("disconnect cleanup failed (%s: %s)", type(exc).__name__, exc)

    # ---- per device loop

    async def run_device(self, dev: DeviceConfig) -> None:
        misses = 0
        while not self.stop_event.is_set():
            try:
                found = await self.find_device(dev)
            except Exception as exc:
                log.warning("%s: scan failed (%s: %s)", dev.name, type(exc).__name__, exc)
                found = None
            if found is None:
                misses += 1
                if misses == 1 or misses % 10 == 0:
                    log.warning("%s: %s not found (scan #%d)", dev.name, dev.mac, misses)
                await self._sleep(self.cfg.reconnect_interval)
                continue
            misses = 0

            cls = await self.resolve_class(dev, *found)
            if cls is None:
                await self._sleep(max(self.cfg.reconnect_interval, 30.0))
                continue

            if self._slots.locked():
                log.warning(
                    "%s: adapter connection limit reached (max_connections=%d, connected: %s); waiting",
                    dev.name, self.cfg.max_connections, ", ".join(sorted(self._connected)) or "-",
                )
            async with self._slots:
                await self._session(dev, cls, found[0])
            await self._sleep(self.cfg.reconnect_interval)

    async def _session(self, dev: DeviceConfig, cls: type[BaseBMS], device: BLEDevice) -> None:
        secret = dev.password if cls.accept_secret else ""
        if dev.password and not cls.accept_secret:
            log.info(
                "%s: ble_connect_password ignored, %s has no BLE authentication in aiobmsble",
                dev.name, module_name(cls),
            )
        bms = cls(device, BMSConfig(keep_alive=True, secret=secret), logger_name=f"aiobmsble.{dev.name}")
        module = module_name(cls)
        connected = False
        reason = "stopped"
        try:
            while not self.stop_event.is_set():
                if connected and not bms.is_connected:
                    # aiobmsble would silently reconnect on the next update;
                    # surface the loss instead (present=False on the ROS side)
                    raise ConnectionError("BLE link lost")
                timeout = self.cfg.update_timeout if connected else self.cfg.connect_timeout
                async with asyncio.timeout(timeout):
                    data = await bms.async_update()
                if not connected:
                    connected = True
                    self._connected.add(dev.name)
                    log.info("%s: connected to %s (%s, %s)", dev.name, device.name, dev.mac, module)
                    self.out.emit("state", name=dev.name, connected=True, bms_type=module, reason="connected")
                    await self._emit_device_info(dev, bms, module)
                self.out.emit("sample", name=dev.name, bms_type=module, data=data)
                await self._sleep(self.cfg.poll_interval)
        except asyncio.CancelledError:
            raise
        except TimeoutError:
            reason = "timeout (%.0f s)" % (self.cfg.update_timeout if connected else self.cfg.connect_timeout)
        except Exception as exc:
            reason = "%s: %s" % (type(exc).__name__, exc)
        finally:
            self._connected.discard(dev.name)
            if reason != "stopped":
                log.warning("%s: %s (%s)", dev.name, "connection lost" if connected else "connect failed", reason)
                if not connected and self._connected:
                    log.warning(
                        "%s: other BMS connected (%s); the adapter may not support more connections",
                        dev.name, ", ".join(sorted(self._connected)),
                    )
            if connected:
                self.out.emit("state", name=dev.name, connected=False, bms_type=module, reason=reason)
            await asyncio.shield(self._safe_disconnect(bms))

    async def _emit_device_info(self, dev: DeviceConfig, bms: BaseBMS, module: str) -> None:
        try:
            async with asyncio.timeout(self.cfg.update_timeout):
                info = await bms.device_info()
        except Exception as exc:  # optional information
            log.info("%s: device info unavailable (%s)", dev.name, type(exc).__name__)
            return
        self.out.emit("device_info", name=dev.name, bms_type=module, info=dict(info))

    async def run(self) -> None:
        tasks = [asyncio.create_task(self.run_device(d), name=d.name) for d in self.cfg.devices]
        await self.stop_event.wait()
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


# --------------------------------------------------------------------------- main


def _watch_stdin(loop: asyncio.AbstractEventLoop, stop: asyncio.Event) -> None:
    """Daemon thread: stdin EOF (node exited or closed the pipe) -> stop."""
    for _ in sys.stdin:
        pass
    loop.call_soon_threadsafe(stop.set)


async def amain(raw_config: dict[str, Any], emitter: Emitter) -> None:
    cfg = BridgeConfig.from_dict(raw_config)
    logging.getLogger("aiobmsble").setLevel(cfg.log_level.upper())
    bridge = Bridge(cfg, emitter)
    loop = asyncio.get_running_loop()
    threading.Thread(target=_watch_stdin, args=(loop, bridge.stop_event), daemon=True).start()
    emitter.emit(
        "ready",
        protocol=PROTOCOL_VERSION,
        aiobmsble=importlib.metadata.version("aiobmsble"),
        bleak=importlib.metadata.version("bleak"),
        devices=[d.name for d in cfg.devices],
    )
    await bridge.run()


def main() -> int:
    emitter = Emitter()
    root = logging.getLogger()
    root.addHandler(EmitLogHandler(emitter))
    root.setLevel(logging.DEBUG)
    log.setLevel(logging.INFO)
    for noisy in ("bleak", "bleak_retry_connector", "asyncio"):
        logging.getLogger(noisy).setLevel(logging.WARNING)

    line = sys.stdin.readline()
    try:
        raw = json.loads(line)
        if raw.get("protocol", PROTOCOL_VERSION) != PROTOCOL_VERSION:
            raise ValueError("protocol version mismatch")
    except ValueError as exc:
        log.critical("invalid config line: %s", exc)
        return 2
    try:
        asyncio.run(amain(raw, emitter))
    except ValueError as exc:
        log.critical("invalid config: %s", exc)
        return 2
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
