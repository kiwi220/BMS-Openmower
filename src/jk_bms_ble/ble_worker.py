"""BLE communication with the JK-BMS in a dedicated thread with its own asyncio loop.

The worker never touches ROS. It reports everything through a thread-safe
``queue.Queue`` so that BLE events cannot block ROS publishing and vice versa.
"""

import asyncio
import logging
import queue
import threading
import time
from collections import namedtuple
from typing import Optional

from . import protocol
from .protocol import ProtocolVersion

# Events put into the output queue. ``stamp`` is time.monotonic().
ConnectionEvent = namedtuple("ConnectionEvent", ["connected", "stamp", "reason"])
DeviceInfoEvent = namedtuple("DeviceInfoEvent", ["info", "protocol", "stamp"])
CellInfoEvent = namedtuple("CellInfoEvent", ["info", "stamp"])

_WAKEUP = object()  # sentinel to wake the frame loop on disconnect / stop


class BleWorker(threading.Thread):
    """Connects to the BMS, keeps the connection alive and emits decoded data."""

    def __init__(
        self,
        mac_address: str,
        out_queue: "queue.Queue",
        logger=None,
        protocol_setting: str = "auto",
        reconnect_interval: float = 5.0,
        scan_timeout: float = 10.0,
        connect_timeout: float = 20.0,
        data_timeout: float = 10.0,
        device_info_timeout: float = 5.0,
        max_request_retries: int = 3,
        password: str = "",
    ) -> None:
        super().__init__(name="jk_bms_ble", daemon=True)
        self._mac = mac_address
        self._out = out_queue
        self._log = logger or logging.getLogger("jk_bms_ble")
        self._protocol_setting = protocol_setting
        self._reconnect_interval = reconnect_interval
        self._scan_timeout = scan_timeout
        self._connect_timeout = connect_timeout
        self._data_timeout = data_timeout
        self._device_info_timeout = device_info_timeout
        self._max_request_retries = max_request_retries
        self._password = password

        self._stop_requested = threading.Event()
        self._loop = None  # type: Optional[asyncio.AbstractEventLoop]
        self._stop_event = None  # type: Optional[asyncio.Event]
        self._frames = None  # type: Optional[asyncio.Queue]
        self._checksum_errors = 0
        self._session_was_connected = False

    # ------------------------------------------------------------------ thread API

    def run(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        try:
            loop.run_until_complete(self._main())
        except Exception as exc:  # never let the thread die silently
            self._log.error("BLE worker crashed: %s", exc)
        finally:
            try:
                loop.run_until_complete(loop.shutdown_asyncgens())
            finally:
                loop.close()

    def stop(self, timeout: float = 5.0) -> None:
        """Request shutdown from any thread and wait for the worker to finish."""
        self._stop_requested.set()
        loop = self._loop
        if loop is not None and not loop.is_closed():
            try:
                loop.call_soon_threadsafe(self._signal_stop)
            except RuntimeError:
                pass  # loop already closed
        if self.is_alive():
            self.join(timeout)

    # ------------------------------------------------------------------ internals

    def _signal_stop(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._frames is not None:
            self._frames.put_nowait(_WAKEUP)

    @property
    def _stopping(self) -> bool:
        return self._stop_requested.is_set()

    def _emit(self, event) -> None:
        """Put an event into the output queue, dropping the oldest entry if full."""
        while True:
            try:
                self._out.put_nowait(event)
                return
            except queue.Full:
                try:
                    self._out.get_nowait()
                except queue.Empty:
                    pass

    async def _sleep_or_stop(self, seconds: float) -> None:
        try:
            await asyncio.wait_for(self._stop_event.wait(), timeout=seconds)
        except asyncio.TimeoutError:
            pass

    async def _main(self) -> None:
        # Created inside the loop: on Python 3.8 asyncio primitives bind to the
        # loop that is current when they are constructed.
        self._stop_event = asyncio.Event()
        if self._stopping:
            return

        if self._password:
            self._log.info(
                "ble_connect_password is set but not sent: the JK02 BLE protocol "
                "(aiobmsble, esphome-jk-bms) has no BLE level authentication; the "
                "PIN is only checked by the JK app for settings changes."
            )

        while not self._stopping:
            reason = "session ended"
            try:
                await self._session()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                reason = "%s: %s" % (type(exc).__name__, exc)
                # bleak raises many different exception types (BleakError,
                # DBus errors, OSError, TimeoutError, ...); all are recoverable.
                self._log.warning("BLE connection to %s failed/lost: %s", self._mac, reason)
            if self._session_was_connected:
                self._emit(ConnectionEvent(False, time.monotonic(), reason))
            if self._stopping:
                break
            self._log.info("Reconnecting to %s in %.1f s", self._mac, self._reconnect_interval)
            await self._sleep_or_stop(self._reconnect_interval)

    async def _session(self) -> None:
        """One scan -> connect -> stream -> disconnect cycle."""
        from bleak import BleakClient, BleakScanner  # imported here: optional for tests

        self._session_was_connected = False
        loop = asyncio.get_event_loop()

        self._log.debug("Scanning for %s (timeout %.0f s)", self._mac, self._scan_timeout)
        device = await BleakScanner.find_device_by_address(self._mac, timeout=self._scan_timeout)
        if device is None:
            raise ConnectionError("device not found during scan")
        if self._stopping:
            return

        self._frames = asyncio.Queue()
        disconnected = asyncio.Event()

        def on_disconnect(_client) -> None:
            # may be called from a bleak/dbus callback; hop onto our loop
            def _set() -> None:
                disconnected.set()
                if self._frames is not None:
                    self._frames.put_nowait(_WAKEUP)
            loop.call_soon_threadsafe(_set)

        assembler = protocol.FrameAssembler()

        def on_notify(_sender, data: bytearray) -> None:
            for frame in assembler.feed(data):
                if not frame.checksum_ok:
                    self._checksum_errors += 1
                    self._log.error(
                        "Invalid checksum in frame type 0x%02X (got 0x%02X, expected 0x%02X, %d errors total)",
                        frame.frame_type,
                        frame.data[protocol.FRAME_LENGTH - 1],
                        protocol.checksum(frame.data[: protocol.FRAME_LENGTH - 1]),
                        self._checksum_errors,
                    )
                    continue
                self._frames.put_nowait(frame)

        async with BleakClient(
            device, disconnected_callback=on_disconnect, timeout=self._connect_timeout
        ) as client:
            notify_char, write_char = self._find_characteristics(client)
            await client.start_notify(notify_char, on_notify)
            self._session_was_connected = True
            self._log.info("Connected to JK-BMS %s (%s)", self._mac, device.name or "unnamed")
            self._emit(ConnectionEvent(True, time.monotonic(), "connected"))

            try:
                await self._stream(client, write_char, disconnected)
            finally:
                if client.is_connected:
                    try:
                        await client.stop_notify(notify_char)
                    except Exception:  # best effort during teardown
                        pass

    @staticmethod
    def _find_characteristics(client):
        notify_char = write_char = None
        for service in client.services:
            for char in service.characteristics:
                if char.uuid.lower() != protocol.CHARACTERISTIC_UUID:
                    continue
                if notify_char is None and "notify" in char.properties:
                    notify_char = char
                if write_char is None and (
                    "write" in char.properties or "write-without-response" in char.properties
                ):
                    write_char = char
        if notify_char is None or write_char is None:
            raise ConnectionError(
                "characteristic %s (notify+write) not found" % protocol.CHARACTERISTIC_UUID
            )
        return notify_char, write_char

    async def _send(self, client, write_char, cmd: int) -> None:
        response = "write-without-response" not in write_char.properties
        data = protocol.build_command(cmd)
        self._log.debug("TX %s", data.hex())
        await client.write_gatt_char(write_char, data, response=response)

    async def _next_frame(self, timeout: float):
        """Next verified frame, or None on timeout / wakeup."""
        try:
            item = await asyncio.wait_for(self._frames.get(), timeout=timeout)
        except asyncio.TimeoutError:
            return None
        return None if item is _WAKEUP else item

    def _resolve_protocol(self, device_info: Optional[protocol.DeviceInfo]) -> ProtocolVersion:
        if self._protocol_setting != "auto":
            return ProtocolVersion(self._protocol_setting)
        if device_info is not None:
            return device_info.protocol
        self._log.warning("No device info received, assuming JK02_32S")
        return ProtocolVersion.JK02_32S

    async def _stream(self, client, write_char, disconnected: asyncio.Event) -> None:
        # 1) Device info (firmware version decides the frame layout)
        await self._send(client, write_char, protocol.CMD_DEVICE_INFO)
        device_info = None
        deadline = time.monotonic() + self._device_info_timeout
        while device_info is None and time.monotonic() < deadline:
            if self._stopping or disconnected.is_set():
                return
            frame = await self._next_frame(deadline - time.monotonic())
            if frame is not None and frame.frame_type == protocol.FRAME_TYPE_DEVICE_INFO:
                device_info = protocol.parse_device_info(frame.data)

        proto = self._resolve_protocol(device_info)
        if device_info is not None:
            self._log.info(
                "BMS model %s, hw %s, sw %s, serial %s -> protocol %s",
                device_info.model, device_info.hw_version, device_info.sw_version,
                device_info.serial_number, proto.value,
            )
            if self._protocol_setting != "auto" and device_info.protocol is not proto:
                self._log.warning(
                    "Configured protocol %s differs from firmware based detection %s",
                    proto.value, device_info.protocol.value,
                )
        self._emit(DeviceInfoEvent(device_info, proto, time.monotonic()))

        # 2) Cell info: after 0x96 the BMS streams frames continuously. If the
        #    stream stalls, re-request; give up (-> reconnect) after N retries.
        await self._send(client, write_char, protocol.CMD_CELL_INFO)
        last_data = time.monotonic()
        retries = 0
        while not self._stopping:
            if disconnected.is_set() or not client.is_connected:
                raise ConnectionError("device disconnected")
            frame = await self._next_frame(1.0)
            now = time.monotonic()
            if frame is not None and frame.frame_type == protocol.FRAME_TYPE_CELL_INFO:
                try:
                    info = protocol.parse_cell_info(frame.data, proto)
                except (protocol.ProtocolError, IndexError, ValueError) as exc:
                    self._log.error("Failed to decode cell info frame: %s", exc)
                    continue
                self._emit(CellInfoEvent(info, now))
                last_data = now
                retries = 0
            elif now - last_data > self._data_timeout:
                if retries >= self._max_request_retries:
                    raise TimeoutError("no cell info for %.0f s" % (now - last_data))
                retries += 1
                self._log.warning(
                    "No cell info for %.0f s, re-requesting (%d/%d)",
                    now - last_data, retries, self._max_request_retries,
                )
                await self._send(client, write_char, protocol.CMD_CELL_INFO)
                last_data = now
