#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from .Controller import Controller, Axis, Button


class Topics:
    def __init__(self, node: Node, mapping: dict, overrides: dict = None):
        self._topics = dict()
        if mapping:
            for key, value in mapping.items():
                if value['type'] == 'Twist':
                    self._topics[key] = TwistTopic(node, value, overrides or {})
                elif value['type'] == 'Bool':
                    self._topics[key] = BoolTopic(node, value)

    def publish(self, controller: Controller):
        for _, topic in self._topics.items():
            topic.publish(controller)

    def __getitem__(self, item):
        return self._buttons

    def __iter__(self):
        return self._buttons.__iter__()

    def __str__(self):
        return '{' + ', '.join(['%s.: %s.' % (key, value.state) for key, value in self._buttons.items()]) + '}'


class TwistTopic:
    def __init__(self, node: Node, params: dict, overrides: dict = None):
        self._node = node
        self.topic = params['topic']
        ovr = overrides or {}

        self.lin_throttle = params['throttle']['lin_throttle_ctrl']
        self.ang_throttle = params['throttle']['ang_throttle_ctrl']
        self._is_turbo = False
        self._before_turbo = 1.0

        self.x = params['data']['linear']['x']
        self.y = params['data']['linear']['y']
        self.z = params['data']['linear']['z']

        self.roll = params['data']['angular']['x']
        self.pitch = params['data']['angular']['y']
        self.yaw = params['data']['angular']['z']

        self._HALT = Twist()
        self._last_message_was_halt = False
        self.clamped = False   # last publish hit a speed ceiling, either axis
        self.throttle_blocked = False   # a D-pad press was refused: already at the ceiling

        # Use override if provided (> 0), otherwise fall back to topics.yaml
        self._lin_throttle_increment = ovr['lin_increment'] if ovr.get('lin_increment', -1.0) > 0 else params['throttle']['lin_increment']
        self._ang_throttle_increment = ovr['ang_increment'] if ovr.get('ang_increment', -1.0) > 0 else params['throttle']['ang_increment']
        self._last_lin_throttle_input = 0
        self._lin_throttle_coef = ovr['start_lin_throttle'] if ovr.get('start_lin_throttle', -1.0) > 0 else 1.0
        self._last_ang_throttle_input = 0
        self._ang_throttle_coef = ovr['start_ang_throttle'] if ovr.get('start_ang_throttle', -1.0) > 0 else 1.0
        self._max_lin_speed = ovr['max_lin_speed'] if ovr.get('max_lin_speed', -1.0) > 0 else params['throttle'].get('max_lin_speed', None)
        self._max_ang_speed = ovr['max_ang_speed'] if ovr.get('max_ang_speed', -1.0) > 0 else params['throttle'].get('max_ang_speed', None)

        try:
            self.publish_multiple_halts = params['publish_multiple_halt']
        except:
            self.publish_multiple_halts = True

        self._publisher = node.create_publisher(Twist, self.topic, 10)

    def publish(self, controller: Controller):
        msg = Twist()
        self.throttle_blocked = False
        lin_throttle_input = self._convert_input(self.lin_throttle, controller)
        self._set_lin_throttle(lin_throttle_input,
                               self._ceiling_coef(self.x, controller, self._max_lin_speed))
        ang_throttle_input = self._convert_input(self.ang_throttle, controller)
        self._set_ang_throttle(ang_throttle_input,
                               self._ceiling_coef(self.yaw, controller, self._max_ang_speed))
        # turbo = self._convert_input(self.turbo, controller)
        # self._set_turbo(turbo)

        raw_x = self._lin_throttle_coef * self._convert_input(self.x, controller)
        msg.linear.x = self._clamp(raw_x, self._max_lin_speed)
        self.clamped = msg.linear.x != raw_x
        msg.linear.y = self._clamp(self._lin_throttle_coef * self._convert_input(self.y, controller), self._max_lin_speed)
        msg.linear.z = self._clamp(self._lin_throttle_coef * self._convert_input(self.z, controller), self._max_lin_speed)
        msg.angular.x = self._clamp(self._ang_throttle_coef * self._convert_input(self.roll, controller), self._max_ang_speed)
        msg.angular.y = self._ang_throttle_coef * self._convert_input(self.pitch, controller)
        raw_z = self._ang_throttle_coef * self._convert_input(self.yaw, controller)
        msg.angular.z = self._clamp(raw_z, self._max_ang_speed)
        self.clamped = self.clamped or msg.angular.z != raw_z
        if msg == self._HALT:
            if self.publish_multiple_halts or not self._last_message_was_halt:
                self._publisher.publish(msg)
            self._last_message_was_halt = True
        else:
            self._publisher.publish(msg)
            self._last_message_was_halt = False

    def _convert_input(self, param, controller: Controller):
        param_type = type(param)
        if param_type == str:
            return float(controller[param].state)
        elif param_type == list:
            return float(sum([controller[val].state for val in param]))
        elif param_type == int or param_type == float:
            return float(param)
        elif param is None:
            return float(0)
        else:
            self._node.get_logger().warn(f'Parameter "%s." should be numeric or None.' % str(param))
            return float(0)

    @staticmethod
    def _clamp(value, max_speed):
        if max_speed is None:
            return value
        return max(-max_speed, min(max_speed, value))

    def _ceiling_coef(self, axis, controller, max_speed):
        """Largest throttle coefficient that still fits under max_speed at full stick."""
        if max_speed is None or not isinstance(axis, str):
            return None
        try:
            scale = abs(float(controller[axis]._scale))
        except (KeyError, AttributeError, TypeError, ValueError):
            return None
        return max_speed / scale if scale else None

    def _step_throttle(self, coef, throttle_input, increment, ceiling):
        """Returns the new coefficient and whether the press was refused."""
        stepped = coef + throttle_input * increment
        if stepped < 0:
            return 0.0, False
        if ceiling is not None and throttle_input > 0 and stepped > ceiling + 1e-9:
            return coef, True        # already as fast as the ceiling allows
        return stepped, False

    def _set_lin_throttle(self, throttle_input, ceiling=None):
        if abs(throttle_input) == 1 and self._last_lin_throttle_input == 0:
            self._lin_throttle_coef, refused = self._step_throttle(
                self._lin_throttle_coef, throttle_input, self._lin_throttle_increment, ceiling)
            self.throttle_blocked = self.throttle_blocked or refused
        self._last_lin_throttle_input = throttle_input

    def _set_ang_throttle(self, throttle_input, ceiling=None):
        if abs(throttle_input) == 1 and self._last_ang_throttle_input == 0:
            self._ang_throttle_coef, refused = self._step_throttle(
                self._ang_throttle_coef, throttle_input, self._ang_throttle_increment, ceiling)
            self.throttle_blocked = self.throttle_blocked or refused
        self._last_ang_throttle_input = throttle_input



class BoolTopic:
    '''Publishes Bool(True) once per press of a controller button.'''
    def __init__(self, node: Node, params: dict):
        self.topic = params['topic']
        self.button = params['button']
        self._last = 0
        self._publisher = node.create_publisher(Bool, self.topic, 10)

    def publish(self, controller: Controller):
        state = int(bool(controller[self.button].state))
        if state and not self._last:
            self._publisher.publish(Bool(data=True))
        self._last = state
