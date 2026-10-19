#!/usr/bin/env python3
import signal
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import Joy, BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import yaml
from modules.Controller import Controller, Axis, Button
from modules.Topics import Topics, TwistTopic, BoolTopic
from modules.PadHeartbeat import PadHeartbeat
from modules.PadIndicator import PadIndicator

# no pad report for this long = link stalled: stop, and ignore the sticks until the operator
# presses the e-stop reset button. Centred sticks are NOT proof: after a pad power cycle the
# driver reports a held stick as 0.0 until it next moves, measured on .39 2026-10-14
PAD_STALE_S = 0.5
LINK_WARN_GAP_S = 2.0      # at most one link-quality tick this often
CEILING_WARN_GAP_S = 2.0   # at most one speed-ceiling cue this often

class Mapper(Node):
    def __init__(self, joy_topic, name="joy_manager"):
        super().__init__(name)
        self._name = name
        self._joy_topic = joy_topic
        self._publish_topics = []

        self._joy_subscriber = self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        controller = self.declare_parameter("controller", "NULL").value
        topics = self.declare_parameter("topics", "NULL").value

        # Robot-specific overrides (passed via launch arguments, -1.0 = use YAML default)
        # LaunchConfiguration passes strings, so convert to float
        self._speed_overrides = {
            'lin_increment': float(self.declare_parameter("lin_increment", -1.0).value),
            'ang_increment': float(self.declare_parameter("ang_increment", -1.0).value),
            'max_lin_speed': float(self.declare_parameter("max_lin_speed", -1.0).value),
            'max_ang_speed': float(self.declare_parameter("max_ang_speed", -1.0).value),
            'start_lin_throttle': float(self.declare_parameter("start_lin_throttle", -1.0).value),
            'start_ang_throttle': float(self.declare_parameter("start_ang_throttle", -1.0).value),
        }

        self._controller = self._configure_controller_mapping(open_yaml(controller)) if controller else None
        self._topics = self._register_topics(open_yaml(topics)) if topics else None

        self._pad = PadHeartbeat(self.get_logger())
        self._pad_stale = False
        self.create_timer(0.05, self._check_pad)

        # light bar and rumble: robot state the operator can see and feel on the pad
        self._feedback = self.declare_parameter("pad_feedback", True).value
        self._indicator = PadIndicator(self.get_logger(), self._feedback)
        self._last_throttle = None
        self._faulted = False
        self._pad_pct = None
        self._estop_engaged = False
        self._wait_centre = False
        self._moving = False
        self._last_joy = 0.0
        self._last_link_warn = 0.0
        self._last_ceiling = 0.0
        self._battery_sub = None
        if self._feedback:
            self.create_subscription(Bool, '/soft_estop/status', self._estop_callback, 10)
            self.create_subscription(Float32MultiArray, '/robot_status',
                                     lambda _: self._indicator.driver_alive(), 10)
            self.create_subscription(DiagnosticArray, '/diagnostics',
                                     self._diagnostics_callback, 10)
            self.create_timer(0.1, self._update_indicator)
            self.create_timer(5.0, self._find_battery_topic)
            self.create_timer(30.0, self._poll_pad_battery)
            self._poll_pad_battery()   # once now, so the state is right from the first second

    def _joy_callback(self, msg: Joy):
        if not self._controller:
            self.get_logger().fatal('Axis and Button mappings must be defined.')
            rclpy.shutdown()
        elif not self._topics:
            self.get_logger().fatal('No topics specified')
            rclpy.shutdown()
        self._last_joy = time.monotonic()
        self._controller.update_states(**{'axes': msg.axes, 'buttons': msg.buttons})

        if self._pad_stale:
            age = self._pad.age()
            if age is None or age > PAD_STALE_S or not self._rearm_pressed():
                return
            self._pad_stale = False
            self.get_logger().info('pad link back and reset pressed: driving re-enabled')
        self._topics.publish(self._controller)
        if self._feedback:
            self._pad_cues()

    def _estop_callback(self, msg: Bool):
        engaged = bool(msg.data)
        if self._estop_engaged and not engaged and not self._centred():
            self._wait_centre = True   # the driver holds the motors until they are centred
        self._estop_engaged = engaged
        self._indicator.set_estop(engaged)

    def _centred(self):
        return self._sticks_centred() if self._topics and self._controller else True

    def _poll_pad_battery(self):
        pct = self._indicator.poll_pad_battery()
        if pct is None or pct == self._pad_pct:
            return
        if self._pad_pct is None:
            self.get_logger().info(f'pad battery {pct}%')
        elif pct <= 20 < self._pad_pct:
            self.get_logger().warn(f'pad battery {pct}%: charge it, a flat pad stops the robot')
        self._pad_pct = pct

    def _diagnostics_callback(self, msg: DiagnosticArray):
        # /diagnostics is a shared bus: another node's warning is not ours to show
        for status in msg.status:
            if not status.name.startswith('roverrobotics_driver'):
                continue
            faulted = status.level >= DiagnosticStatus.ERROR
            if faulted != self._faulted:
                self._faulted = faulted
                if faulted:
                    self.get_logger().warn(f'{status.message}: the robot will not move')
                else:
                    self.get_logger().info(f'cleared: {status.message}')
            self._indicator.set_hardware_fault(faulted)

    def _battery_callback(self, msg: BatteryState):
        # percentage is 0-100 in this driver. Never infer charging from current: on a
        # VESC robot that is one motor's draw or regen, so the driver reports UNKNOWN
        soc = None if msg.percentage != msg.percentage else msg.percentage / 100.0
        charging = msg.power_supply_status in (BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                                               BatteryState.POWER_SUPPLY_STATUS_FULL)
        self._indicator.set_battery(soc, charging)

    def _find_battery_topic(self):
        # the driver names it rover_<type>/battery_status, so look it up by type
        if self._battery_sub:
            return
        for name, types in self.get_topic_names_and_types():
            if 'sensor_msgs/msg/BatteryState' in types:
                self._battery_sub = self.create_subscription(
                    BatteryState, name, self._battery_callback, 10)
                self.get_logger().info(f'pad feedback: battery from {name}')
                return

    def _update_indicator(self):
        # only trust the moving flag while joy messages are actually arriving
        if time.monotonic() - self._last_joy > PAD_STALE_S:
            self._moving = False
        self._indicator.set_moving(self._moving)
        if self._wait_centre and self._centred():
            self._wait_centre = False
        self._indicator.set_wait_centre(self._wait_centre)
        self._indicator.set_link_stop(self._pad_stale)
        self._indicator.set_pad_stale(self._pad_stale)
        now = time.monotonic()
        if self._pad.degraded() and now - self._last_link_warn > LINK_WARN_GAP_S:
            self._last_link_warn = now
            self._indicator.tick_link_warn()
        self._indicator.update()

    def _pad_cues(self):
        # throttle step and speed-ceiling cues, straight off the topics we just published
        for topic in self._topics._topics.values():
            if not isinstance(topic, TwistTopic):
                continue
            self._moving = not topic._last_message_was_halt
            throttle = (topic._lin_throttle_coef, topic._ang_throttle_coef)
            if self._last_throttle is not None and throttle != self._last_throttle:
                self._indicator.tick_throttle()
            self._last_throttle = throttle
            now = time.monotonic()
            if topic.throttle_blocked:
                self._last_ceiling = now
                self._indicator.tick_ceiling()   # the D-pad asked past the ceiling
            elif topic.clamped and now - self._last_ceiling > CEILING_WARN_GAP_S:
                self._last_ceiling = now
                self._indicator.tick_ceiling()
            return

    def _check_pad(self):
        age = self._pad.age()
        if not self._topics or self._pad_stale or age is None or age <= PAD_STALE_S:
            return
        self._pad_stale = True
        self.get_logger().warn(f'no pad report for {age:.2f} s: stopping; sticks ignored until '
                               'the link is back and the reset button is pressed')
        for topic in self._topics._topics.values():
            if isinstance(topic, TwistTopic):
                topic._publisher.publish(Twist())

    def _rearm_pressed(self):
        # whatever button the config binds to the e-stop reset re-arms the pad path
        if not self._topics or not self._controller:
            return False
        for topic in self._topics._topics.values():
            if isinstance(topic, BoolTopic) and topic.topic.rstrip('/').endswith('reset'):
                try:
                    return bool(self._controller[topic.button].state)
                except KeyError:
                    return False
        return False

    def _sticks_centred(self):
        for topic in self._topics._topics.values():
            if isinstance(topic, TwistTopic):
                for ax in (topic.x, topic.yaw):
                    if abs(topic._convert_input(ax, self._controller)) > 0.05:
                        return False
        return True

    def _configure_controller_mapping(self, button_mappings: dict):
        return Controller(button_mappings, self.get_logger())

    def _register_topics(self, topics: dict):
        return Topics(self, topics, self._speed_overrides)

    def update_topics(self):
        for topic in self._topics:
            topic.publish_topic(self.state)


def blank_pad(mapper):
    # The light bar latches in hardware: if this process dies without clearing it,
    # the pad keeps showing the last colour and a dead robot still reads as green.
    try:
        mapper._indicator.close()
    except Exception:
        pass


def open_yaml(file_name):
    with open(file_name) as stream:
        return yaml.safe_load(stream)

def main(args=None):
    # systemd and launch send SIGTERM; without a handler Python exits without unwinding
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    rclpy.init(args=args)
    mapper = Mapper('/joy')
    try:
        rclpy.spin(mapper)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        blank_pad(mapper)

if __name__ == '__main__':
    main()
