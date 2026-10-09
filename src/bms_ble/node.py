"""ROS 1 (rospy) node: BLE battery management systems -> sensor_msgs/BatteryState.

Threading model:
    * bridge process (Python >= 3.12, aiobmsble + bleak): all BLE I/O, one
      asyncio task per BMS.
    * supervisor thread (BridgeClient): reads the bridge's JSON lines into a
      thread-safe queue.Queue, restarts the bridge if it dies.
    * rospy main thread: drains the queue at publish_rate_hz and publishes.
      It never waits for BLE.
"""

import os
import queue
import sys
import time
from typing import Dict, List, Optional

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import BatteryState

from . import battery_logic as bl
from . import conversion
from .bridge_client import BridgeClient
from .config import (COMBINED_NAME, DeviceConfig, parse_bms_list, parse_pack_connection, parse_positive,
                     parse_xbot_rate, parse_xbot_sensor_bms)
from .model import BmsSample
from .xbot_sensors import XbotSensorPublisher

# /ll/power is published by mower_comms and carries the charger voltage used for
# dock detection; a second publisher there would break docking.
FORBIDDEN_STATUS_TOPICS = ("/ll/power",)
DEFAULT_BRIDGE_PYTHON = "~/.local/share/bms_ble/venv/bin/python"

BRIDGE_LOG = {
    "debug": rospy.logdebug,
    "info": rospy.loginfo,
    "warning": rospy.logwarn,
    "error": rospy.logerr,
    "critical": rospy.logfatal,
}


class RospyLogger:
    """logging.Logger-like adapter so the bridge client logs to rosout."""

    debug = staticmethod(rospy.logdebug)
    info = staticmethod(rospy.loginfo)
    warning = staticmethod(rospy.logwarn)
    error = staticmethod(rospy.logerr)


class DeviceState:
    """Runtime state of one BMS, only touched from the ROS main thread."""

    def __init__(self, cfg: DeviceConfig) -> None:
        self.cfg = cfg
        self.connected = False
        self.disconnected_since = time.monotonic()  # "never connected" counts as outage
        self.sample: Optional[BmsSample] = None
        self.sample_stamp: Optional[float] = None
        self.bms_type = ""
        self.device_info: Dict[str, str] = {}
        self.stale_reported = False
        self.cell_count_warned = False
        self.pub: Optional[rospy.Publisher] = None

    def is_stale(self, now: float, timeout: float) -> bool:
        """Outage longer than timeout, or no fresh data while connected."""
        if not self.connected and now - self.disconnected_since > timeout:
            return True
        return self.sample_stamp is None or now - self.sample_stamp > timeout


def _default_bridge_script() -> str:
    try:
        import rospkg

        return os.path.join(rospkg.RosPack().get_path("bms_ble"), "bridge", "bms_bridge.py")
    except Exception:
        return os.path.join(os.path.dirname(__file__), "..", "..", "bridge", "bms_bridge.py")


class BmsNode:
    def __init__(self) -> None:
        legacy = {k: rospy.get_param("~" + k) for k in
                  ("bms_mac_address", "cell_count", "nominal_capacity_ah", "ble_connect_password")
                  if rospy.has_param("~" + k)}
        self.devices = [DeviceState(c) for c in parse_bms_list(rospy.get_param("~bms_list", []), legacy)]
        self.by_name = {d.cfg.name: d for d in self.devices}
        if legacy and rospy.has_param("~bms_list"):
            rospy.logwarn("Both bms_list and legacy single-BMS parameters set; using bms_list")

        self.publish_rate = parse_positive("publish_rate_hz", rospy.get_param("~publish_rate_hz", 1.0))
        self.stale_timeout = parse_positive("stale_timeout_s", rospy.get_param("~stale_timeout_s", 30.0))
        self.frame_id = str(rospy.get_param("~frame_id", "battery"))
        self.pack_connection = parse_pack_connection(rospy.get_param("~pack_connection", ""))

        self._setup_publishers()

        self.queue = queue.Queue(maxsize=500)
        bridge_python = os.path.expanduser(str(rospy.get_param("~bridge_python", DEFAULT_BRIDGE_PYTHON)))
        bridge_script = os.path.expanduser(str(rospy.get_param("~bridge_script", "") or _default_bridge_script()))
        reconnect = parse_positive("reconnect_interval_s", rospy.get_param("~reconnect_interval_s", 5.0))
        self.bridge = BridgeClient(
            [bridge_python, "-u", bridge_script],
            {
                "devices": [d.cfg.bridge_dict() for d in self.devices],
                "poll_interval": 1.0 / self.publish_rate,
                "reconnect_interval": reconnect,
                "scan_timeout": parse_positive("scan_timeout_s", rospy.get_param("~scan_timeout_s", 10.0)),
                "connect_timeout": parse_positive("connect_timeout_s", rospy.get_param("~connect_timeout_s", 45.0)),
                "update_timeout": parse_positive("update_timeout_s", rospy.get_param("~update_timeout_s", 20.0)),
                "max_connections": parse_positive(
                    "max_connections", rospy.get_param("~max_connections", 4), integer=True
                ),
                "log_level": str(rospy.get_param("~bridge_log_level", "WARNING")),
            },
            self.queue,
            logger=RospyLogger(),
            restart_interval=reconnect,
        )

    # ------------------------------------------------------------------ setup

    def _setup_publishers(self) -> None:
        for dev in self.devices:
            dev.pub = rospy.Publisher("battery_state/" + dev.cfg.name, BatteryState, queue_size=1)

        primary = [d for d in self.devices if d.cfg.primary]
        self.primary = primary[0] if primary else None
        self.primary_pub = None
        if self.primary:
            topic = str(rospy.get_param("~primary_topic", "/battery_state"))
            self.primary_pub = rospy.Publisher(topic, BatteryState, queue_size=1)
        else:
            rospy.logwarn("No BMS marked primary: nothing is published on /battery_state")

        self.combined_pub = None
        if self.pack_connection and len(self.devices) > 1:
            self.combined_pub = rospy.Publisher("battery_state/" + COMBINED_NAME, BatteryState, queue_size=1)
        elif len(self.devices) > 1:
            rospy.loginfo("pack_connection not set: battery_state/combined is not published")
        elif self.pack_connection:
            rospy.logwarn("pack_connection is ignored with a single BMS")

        self.diag_pub = None
        if bool(rospy.get_param("~publish_diagnostics", True)):
            self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        self.xbot = None
        if bool(rospy.get_param("~publish_xbot_sensors", True)):
            self._setup_xbot_sensors()

        self.bms_pub = self.bms_msg_cls = None
        status_topic = str(rospy.get_param("~openmower_status_topic", "")).strip()
        if status_topic:
            self._setup_openmower_publisher(status_topic)

    def _setup_openmower_publisher(self, topic: str) -> None:
        if rospy.resolve_name(topic) in FORBIDDEN_STATUS_TOPICS:
            rospy.logerr(
                "openmower_status_topic=%s refused: this topic is owned by mower_comms "
                "(charger voltage = dock detection). Use /ll/bms (mower_msgs/Bms) instead.",
                topic,
            )
            return
        if self.primary is None:
            rospy.logerr("openmower_status_topic set but no BMS is primary; not publishing it")
            return
        try:
            from mower_msgs.msg import Bms  # only available inside open_mower_ros
        except ImportError:
            rospy.logerr(
                "openmower_status_topic=%s set, but mower_msgs is not importable "
                "(source the open_mower_ros workspace)", topic,
            )
            return
        self.bms_msg_cls = Bms
        self.bms_pub = rospy.Publisher(topic, Bms, queue_size=1)
        rospy.loginfo("Publishing mower_msgs/Bms of '%s' on %s", self.primary.cfg.name, self.bms_pub.resolved_name)

    def _setup_xbot_sensors(self) -> None:
        rate = parse_xbot_rate(rospy.get_param("~xbot_sensors_rate_hz", 1.0))
        devices = parse_xbot_sensor_bms(rospy.get_param("~xbot_sensor_bms", ""), [d.cfg for d in self.devices])
        if not devices:
            rospy.logwarn("xbot sensors disabled: no BMS is primary; set xbot_sensor_bms to a name or 'all'")
            return
        try:
            from xbot_msgs.msg import SensorDataDouble, SensorDataString, SensorInfo  # open_mower_ros only
        except ImportError:
            rospy.logwarn(
                "xbot sensors disabled: xbot_msgs is not importable (source the open_mower_ros workspace "
                "or set publish_xbot_sensors: false)"
            )
            return
        self.xbot = XbotSensorPublisher(devices, rospy.Publisher, SensorInfo, SensorDataDouble, SensorDataString, rate)
        rospy.loginfo(
            "Publishing %d xbot_monitoring sensors for %s at %.1f Hz",
            len(self.xbot.sensor_ids), ", ".join(d.name for d in devices), rate,
        )

    # ------------------------------------------------------------------ main loop

    def spin(self) -> None:
        rospy.on_shutdown(self._shutdown)
        self.bridge.start()
        for d in self.devices:
            rospy.loginfo(
                "BMS '%s': type %s, MAC %s, %d cells, %.1f Ah%s",
                d.cfg.name, d.cfg.type, d.cfg.mac, d.cfg.cell_count, d.cfg.nominal_capacity_ah,
                " (primary)" if d.cfg.primary else "",
            )
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self._drain_queue()
            self._publish()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _shutdown(self) -> None:
        rospy.loginfo("Stopping BMS bridge")
        self.bridge.stop(timeout=10.0)

    # ------------------------------------------------------------------ events

    def _drain_queue(self) -> None:
        while True:
            try:
                event = self.queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._handle(event)
            except Exception as exc:  # a malformed event must never stop publishing
                rospy.logerr("Failed to handle bridge event %s: %s", event.get("event"), exc)

    def _handle(self, ev: Dict) -> None:
        kind = ev.get("event")
        if kind == "log":
            log = BRIDGE_LOG.get(ev.get("level", "info"), rospy.loginfo)
            log("[bridge] %s", ev.get("msg", ""))
            return
        if kind == "ready":
            rospy.loginfo("BMS bridge ready (aiobmsble %s, bleak %s)", ev.get("aiobmsble"), ev.get("bleak"))
            return
        if kind == "bridge_exit":
            rospy.logerr("BMS bridge exited (code %s); restarting", ev.get("code"))
            for dev in self.devices:
                self._set_disconnected(dev, ev.get("rx", time.monotonic()), "bridge exited")
            return

        dev = self.by_name.get(ev.get("name", ""))
        if dev is None:
            rospy.logdebug("Event for unknown BMS: %s", ev)
            return
        if kind == "sample":
            dev.bms_type = ev.get("bms_type", dev.bms_type)
            dev.sample = BmsSample.from_bridge(dev.bms_type, ev.get("data") or {})
            dev.sample_stamp = ev.get("rx", time.monotonic())
            self._check_cells(dev)
        elif kind == "state":
            if ev.get("connected"):
                dev.connected = True
                dev.stale_reported = False
                dev.bms_type = ev.get("bms_type", dev.bms_type)
                rospy.loginfo("BMS '%s' connected (%s)", dev.cfg.name, dev.bms_type)
            else:
                self._set_disconnected(dev, ev.get("rx", time.monotonic()), ev.get("reason", ""))
        elif kind == "device_info":
            dev.device_info = {str(k): str(v) for k, v in (ev.get("info") or {}).items()}
            rospy.loginfo("BMS '%s' info: %s", dev.cfg.name, dev.device_info)

    def _set_disconnected(self, dev: DeviceState, stamp: float, reason: str) -> None:
        if not dev.connected:
            return
        dev.connected = False
        dev.disconnected_since = stamp
        rospy.logwarn(
            "BMS '%s' connection lost (%s); publishing last known values with present=False",
            dev.cfg.name, reason,
        )

    def _check_cells(self, dev: DeviceState) -> None:
        n = len(dev.sample.cell_voltages)
        if n != dev.cfg.cell_count and not dev.cell_count_warned:
            dev.cell_count_warned = True
            rospy.logwarn("BMS '%s' reports %d cells but cell_count=%d", dev.cfg.name, n, dev.cfg.cell_count)

    # ------------------------------------------------------------------ publishing

    def _publish(self) -> None:
        now = time.monotonic()
        stamp = rospy.Time.now()
        states: List[BatteryState] = []
        statuses = []
        stale_by_name: Dict[str, bool] = {}
        for dev in self.devices:
            stale = dev.is_stale(now, self.stale_timeout)
            stale_by_name[dev.cfg.name] = stale
            if stale and dev.sample is not None and not dev.stale_reported:
                dev.stale_reported = True
                rospy.logwarn(
                    "BMS '%s': no data for more than %.0f s, power_supply_health=UNKNOWN",
                    dev.cfg.name, self.stale_timeout,
                )
            if not dev.connected:
                rospy.logwarn_throttle(30.0, "BMS '%s' not connected, retrying in background" % dev.cfg.name)

            msg = conversion.battery_state(
                BatteryState, dev.cfg, dev.sample, dev.connected, stale, stamp,
                self.frame_id, dev.device_info.get("serial_number", ""),
            )
            dev.pub.publish(msg)
            states.append(msg)
            if dev is self.primary:
                self.primary_pub.publish(msg)
                if self.bms_pub is not None and dev.sample is not None:
                    self.bms_pub.publish(conversion.bms_message(self.bms_msg_cls, dev.sample, stale, stamp))
            statuses.append(self._diagnostic(dev, stale))

        if self.combined_pub is not None:
            if all(d.sample is not None for d in self.devices):
                self.combined_pub.publish(
                    conversion.combined_state(BatteryState, states, self.pack_connection, stamp, self.frame_id)
                )
            else:
                rospy.loginfo_throttle(30.0, "battery_state/combined waits for data from all BMS")

        if self.xbot is not None and self.xbot.due(now):
            for dev in self.devices:
                if dev.cfg in self.xbot.devices:
                    added = self.xbot.update(dev.cfg, dev.sample)
                    if added:
                        rospy.loginfo("xbot sensors added: %s", ", ".join(added))
                    self.xbot.publish(dev.cfg, dev.sample, dev.connected, stale_by_name[dev.cfg.name], stamp)

        if self.diag_pub is not None:
            array = DiagnosticArray()
            array.header.stamp = stamp
            array.status = statuses
            self.diag_pub.publish(array)

    def _diagnostic(self, dev: DeviceState, stale: bool) -> DiagnosticStatus:
        status = DiagnosticStatus(name="bms_ble: %s" % dev.cfg.name, hardware_id=dev.cfg.mac)
        values = [("connected", dev.connected), ("bms_type", dev.bms_type or dev.cfg.type)]
        values += sorted(dev.device_info.items())
        s = dev.sample
        if s is None:
            status.level, status.message = DiagnosticStatus.STALE, "No data received yet"
        else:
            def fmt(v, spec="%.3f"):
                return "NA" if v is None else spec % v

            values += [
                ("voltage_v", fmt(s.voltage)),
                ("current_a", fmt(s.current)),
                ("soc_pct", fmt(s.soc, "%.0f")),
                ("soh_pct", fmt(s.soh, "%.0f")),
                ("cells_v", " ".join("%.3f" % v for v in s.cell_voltages) or "NA"),
                ("temp_mosfet_c", fmt(s.mosfet_temperature, "%.1f")),
                ("temp_sensors_c", " ".join("%.1f" % v for v in s.sensor_temperatures) or "NA"),
                ("charge_mosfet", "NA" if s.charge_mosfet is None else ("on" if s.charge_mosfet else "off")),
                ("discharge_mosfet", "NA" if s.discharge_mosfet is None else ("on" if s.discharge_mosfet else "off")),
                ("cycles", "NA" if s.cycles is None else s.cycles),
                ("problem_code", "0x%X" % s.problem_code),
            ]
            errors = bl.error_names(s.vendor, s.problem_code)
            if errors:
                values.append(("errors", ", ".join(errors)))
            health = bl.power_supply_health(s.vendor, s.problem_code, s.problem)
            if stale:
                status.level, status.message = DiagnosticStatus.STALE, "No data for > %.0f s" % self.stale_timeout
            elif health != bl.POWER_SUPPLY_HEALTH_GOOD:
                status.level = DiagnosticStatus.ERROR
                status.message = ", ".join(errors) or "BMS reports problem 0x%X" % s.problem_code
            elif not dev.connected:
                status.level, status.message = DiagnosticStatus.WARN, "BLE disconnected, showing last values"
            elif errors:
                status.level, status.message = DiagnosticStatus.WARN, ", ".join(errors)
            else:
                status.level, status.message = DiagnosticStatus.OK, "OK"
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values]
        return status


def main() -> None:
    rospy.init_node("bms_ble")
    try:
        node = BmsNode()
    except ValueError as exc:
        rospy.logfatal("Invalid configuration: %s", exc)
        sys.exit(1)
    node.spin()
