"""ROS 1 (rospy) node publishing JK-BMS BLE data as sensor_msgs/BatteryState.

Threading model:
    * BleWorker thread: bleak + its own asyncio loop, pushes events into a
      thread-safe queue.Queue.
    * rospy main thread: drains the queue at ``publish_rate_hz`` and publishes.
      Nothing in this thread ever awaits BLE I/O.
"""

import json
import queue
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import BatteryState

from . import battery_logic as bl
from .ble_worker import BleWorker, CellInfoEvent, ConnectionEvent, DeviceInfoEvent

NAN = float("nan")

# /ll/power is published by mower_comms and carries charger voltage used for
# dock detection; a second publisher there would break docking.
FORBIDDEN_STATUS_TOPICS = ("/ll/power",)


class RospyLogger:
    """logging.Logger-like adapter so the BLE worker logs to rosout."""

    debug = staticmethod(rospy.logdebug)
    info = staticmethod(rospy.loginfo)
    warning = staticmethod(rospy.logwarn)
    error = staticmethod(rospy.logerr)


class JkBmsNode:
    def __init__(self) -> None:
        self.mac = str(rospy.get_param("~bms_mac_address", "")).strip()
        if not self.mac:
            raise ValueError("parameter ~bms_mac_address is required")
        self.publish_rate = float(rospy.get_param("~publish_rate_hz", 1.0))
        if self.publish_rate <= 0:
            raise ValueError("parameter ~publish_rate_hz must be > 0")
        self.cell_count = int(rospy.get_param("~cell_count", 7))
        self.nominal_capacity = float(rospy.get_param("~nominal_capacity_ah", 10.0))
        self.stale_timeout = float(rospy.get_param("~stale_timeout_s", 30.0))
        self.frame_id = str(rospy.get_param("~frame_id", "battery"))
        password = str(rospy.get_param("~ble_connect_password", ""))
        protocol_setting = str(rospy.get_param("~protocol", "auto"))
        if protocol_setting not in ("auto", "JK02_32S", "JK02_24S"):
            raise ValueError("parameter ~protocol must be auto, JK02_32S or JK02_24S")

        battery_topic = str(rospy.get_param("~battery_state_topic", "/battery_state"))
        self.battery_pub = rospy.Publisher(battery_topic, BatteryState, queue_size=1)

        self.diag_pub = None
        if bool(rospy.get_param("~publish_diagnostics", True)):
            self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)

        self.bms_pub = None
        self.bms_msg_cls = None
        status_topic = str(rospy.get_param("~openmower_status_topic", "")).strip()
        if status_topic:
            self._setup_openmower_publisher(status_topic)

        # State, only touched from the ROS main thread
        self.queue = queue.Queue(maxsize=100)
        self.connected = False
        self.disconnected_since = time.monotonic()  # "never connected" counts as outage
        self.last_info = None
        self.last_info_stamp = None
        self.device_info = None
        self.stale_reported = False
        self.cell_count_warned = False

        self.worker = BleWorker(
            self.mac,
            self.queue,
            logger=RospyLogger(),
            protocol_setting=protocol_setting,
            reconnect_interval=float(rospy.get_param("~reconnect_interval_s", 5.0)),
            data_timeout=float(rospy.get_param("~data_timeout_s", 10.0)),
            password=password,
        )

    def _setup_openmower_publisher(self, topic: str) -> None:
        if rospy.resolve_name(topic) in FORBIDDEN_STATUS_TOPICS:
            rospy.logerr(
                "openmower_status_topic=%s refused: this topic is owned by mower_comms "
                "(charger voltage = dock detection). Use /ll/bms (mower_msgs/Bms) instead.",
                topic,
            )
            return
        try:
            from mower_msgs.msg import Bms  # only available inside open_mower_ros
        except ImportError:
            rospy.logerr(
                "openmower_status_topic=%s set, but mower_msgs is not importable "
                "(source the open_mower_ros workspace). Only %s is published.",
                topic, self.battery_pub.resolved_name,
            )
            return
        self.bms_msg_cls = Bms
        self.bms_pub = rospy.Publisher(topic, Bms, queue_size=1)
        rospy.loginfo("Additionally publishing mower_msgs/Bms on %s", self.bms_pub.resolved_name)

    # ------------------------------------------------------------------ main loop

    def spin(self) -> None:
        rospy.on_shutdown(self._shutdown)
        self.worker.start()
        rospy.loginfo(
            "JK-BMS node started: MAC %s, %.1f Hz, %d cells, %.1f Ah nominal",
            self.mac, self.publish_rate, self.cell_count, self.nominal_capacity,
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
        rospy.loginfo("Shutting down JK-BMS BLE worker")
        self.worker.stop(timeout=5.0)

    def _drain_queue(self) -> None:
        while True:
            try:
                event = self.queue.get_nowait()
            except queue.Empty:
                return
            if isinstance(event, CellInfoEvent):
                self.last_info = event.info
                self.last_info_stamp = event.stamp
            elif isinstance(event, ConnectionEvent):
                self._on_connection(event)
            elif isinstance(event, DeviceInfoEvent):
                self.device_info = event.info

    def _on_connection(self, event: ConnectionEvent) -> None:
        if event.connected:
            self.connected = True
            self.stale_reported = False
            rospy.loginfo("JK-BMS %s connected", self.mac)
        else:
            if self.connected:
                self.disconnected_since = event.stamp
            self.connected = False
            rospy.logwarn(
                "JK-BMS %s connection lost (%s); publishing last known values with present=False",
                self.mac, event.reason,
            )

    # ------------------------------------------------------------------ state

    def _is_stale(self, now: float) -> bool:
        """Outage longer than stale_timeout, or no fresh data while connected."""
        if not self.connected and now - self.disconnected_since > self.stale_timeout:
            return True
        return self.last_info_stamp is None or now - self.last_info_stamp > self.stale_timeout

    def _cell_voltages(self, info):
        cells = list(info.cell_voltages[: self.cell_count])
        if info.enabled_cell_count != self.cell_count and not self.cell_count_warned:
            self.cell_count_warned = True
            rospy.logwarn(
                "BMS reports %d enabled cells but cell_count=%d", info.enabled_cell_count, self.cell_count
            )
        cells += [NAN] * (self.cell_count - len(cells))
        return cells

    # ------------------------------------------------------------------ publishing

    def _publish(self) -> None:
        now = time.monotonic()
        stale = self._is_stale(now)
        if stale and not self.stale_reported and self.last_info is not None:
            self.stale_reported = True
            rospy.logwarn(
                "No JK-BMS data for more than %.0f s, power_supply_health=UNKNOWN", self.stale_timeout
            )
        if not self.connected:
            rospy.logwarn_throttle(30.0, "JK-BMS %s not connected, retrying in background" % self.mac)

        stamp = rospy.Time.now()
        self.battery_pub.publish(self._battery_state(stamp, stale))
        if self.bms_pub is not None and self.last_info is not None:
            self.bms_pub.publish(self._bms_msg(stamp, stale))
        if self.diag_pub is not None:
            self.diag_pub.publish(self._diagnostics(stamp, stale))

    def _battery_state(self, stamp, stale: bool) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = self.connected
        msg.design_capacity = self.nominal_capacity
        msg.serial_number = self.device_info.serial_number if self.device_info else ""

        info = self.last_info
        if info is None:
            # nothing received yet: NaN = "not measured" per BatteryState.msg
            msg.voltage = msg.current = msg.charge = msg.percentage = msg.temperature = NAN
            msg.capacity = NAN
            msg.cell_voltage = [NAN] * self.cell_count
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            return msg

        capacity = info.full_charge_capacity if info.full_charge_capacity > 0 else self.nominal_capacity
        percentage = min(max(info.state_of_charge / 100.0, 0.0), 1.0)
        msg.voltage = info.total_voltage
        msg.current = info.current
        msg.capacity = capacity
        msg.charge = info.remaining_capacity if info.remaining_capacity > 0 else percentage * capacity
        msg.percentage = percentage
        msg.temperature = info.mosfet_temperature if info.mosfet_temperature is not None else NAN
        msg.cell_voltage = self._cell_voltages(info)
        msg.power_supply_status = bl.power_supply_status(info.current, info.state_of_charge)
        msg.power_supply_health = bl.power_supply_health(info.error_bitmask, stale)
        return msg

    def _bms_msg(self, stamp, stale: bool):
        """mower_msgs/Bms as consumed by mower_logic via /ll/bms."""
        info = self.last_info
        msg = self.bms_msg_cls()
        msg.stamp = stamp
        if stale:
            # mower_logic keeps the last message forever; NaN makes
            # utils::GetFirstValid() skip it instead of trusting old voltage.
            msg.voltage = msg.current = msg.relative_state_of_charge = NAN
            msg.remaining_capacity = msg.full_charge_capacity = msg.temperature = NAN
            msg.battery_status = "Stale"
            return msg
        status = []
        if info.state_of_charge >= 100:
            status.append("Fully charged")
        if info.current < -bl.CURRENT_THRESHOLD_A:
            status.append("Discharging")
        status += ["ALARM: %s" % e for e in info.errors]
        msg.voltage = info.total_voltage
        msg.current = info.current
        msg.relative_state_of_charge = float(info.state_of_charge)  # percent, SMBus convention
        msg.remaining_capacity = info.remaining_capacity
        msg.full_charge_capacity = info.full_charge_capacity
        msg.cycle_count = min(info.cycle_count, 0xFFFF)
        msg.temperature = info.mosfet_temperature if info.mosfet_temperature is not None else NAN
        msg.battery_status = ", ".join(status)
        msg.extra_data = json.dumps({
            "cells": [round(v, 3) for v in info.cell_voltages],
            "t1": info.temperatures[0],
            "t2": info.temperatures[1],
            "charge_mosfet": info.charge_mosfet,
            "discharge_mosfet": info.discharge_mosfet,
            "soh": info.state_of_health,
            "error_bitmask": info.error_bitmask,
        })
        return msg

    def _diagnostics(self, stamp, stale: bool) -> DiagnosticArray:
        status = DiagnosticStatus(name="jk_bms_ble: Battery", hardware_id=self.mac)
        info = self.last_info
        values = [("connected", self.connected)]
        if self.device_info:
            values += [
                ("model", self.device_info.model),
                ("sw_version", self.device_info.sw_version),
                ("serial_number", self.device_info.serial_number),
            ]
        if info is None:
            status.level, status.message = DiagnosticStatus.STALE, "No data received yet"
        else:
            def fmt_temp(t):
                return "NA" if t is None else "%.1f" % t

            cells = info.cell_voltages or [NAN]
            values += [
                ("voltage_v", "%.3f" % info.total_voltage),
                ("current_a", "%.3f" % info.current),
                ("soc_pct", info.state_of_charge),
                ("soh_pct", info.state_of_health),
                ("cells_v", " ".join("%.3f" % v for v in info.cell_voltages)),
                ("cell_delta_v", "%.3f" % (max(cells) - min(cells))),
                ("temp_mosfet_c", fmt_temp(info.mosfet_temperature)),
                ("temp_t1_c", fmt_temp(info.temperatures[0])),
                ("temp_t2_c", fmt_temp(info.temperatures[1])),
                ("charge_mosfet", "on" if info.charge_mosfet else "off"),
                ("discharge_mosfet", "on" if info.discharge_mosfet else "off"),
                ("cycles", info.cycle_count),
                ("error_bitmask", "0x%08X" % info.error_bitmask),
                ("errors", ", ".join(info.errors) or "none"),
            ]
            health = bl.power_supply_health(info.error_bitmask)
            if stale:
                status.level, status.message = DiagnosticStatus.STALE, "Data older than %.0f s" % self.stale_timeout
            elif health != bl.POWER_SUPPLY_HEALTH_GOOD:
                status.level, status.message = DiagnosticStatus.ERROR, ", ".join(info.errors)
            elif not self.connected:
                status.level, status.message = DiagnosticStatus.WARN, "BLE disconnected, showing last values"
            elif info.errors:
                status.level, status.message = DiagnosticStatus.WARN, ", ".join(info.errors)
            else:
                status.level, status.message = DiagnosticStatus.OK, "OK"
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values]
        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status = [status]
        return array


def main() -> None:
    rospy.init_node("jk_bms_ble")
    try:
        node = JkBmsNode()
    except ValueError as exc:
        rospy.logfatal("Invalid configuration: %s", exc)
        raise SystemExit(1)
    node.spin()
