#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from .Controller import Controller, Axis, Button


class Topics:
    def __init__(self, node: Node, mapping: dict):
        self._topics = dict()
        if mapping:
            for key, value in mapping.items():
                if value['type'] == 'Twist':
                    self._topics[key] = TwistTopic(node, value)

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
    def __init__(self, node: Node, params: dict):
        self._node = node
        self.topic = params['topic']

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

        self._lin_throttle_increment = params['throttle']['lin_increment']
        self._ang_throttle_increment = params['throttle']['ang_increment']
        self._last_lin_throttle_input = 0
        self._lin_throttle_coef = 1.0
        self._last_ang_throttle_input = 0
        self._ang_throttle_coef = 1.0

        try:
            self.publish_multiple_halts = params['publish_multiple_halt']
        except:
            self.publish_multiple_halts = True

        self._publisher = node.create_publisher(Twist, self.topic, 10)

    def publish(self, controller: Controller):
        msg = Twist()
        lin_throttle_input = self._convert_input(self.lin_throttle, controller)
        self._set_lin_throttle(lin_throttle_input)
        ang_throttle_input = self._convert_input(self.ang_throttle, controller)
        self._set_ang_throttle(ang_throttle_input)
        # turbo = self._convert_input(self.turbo, controller)
        # self._set_turbo(turbo)

        msg.linear.x = self._lin_throttle_coef * self._convert_input(self.x, controller)
        msg.linear.y = self._lin_throttle_coef * self._convert_input(self.y, controller)
        msg.linear.z = self._lin_throttle_coef * self._convert_input(self.z, controller)
        msg.angular.x = self._ang_throttle_coef * self._convert_input(self.roll, controller)
        msg.angular.y = self._ang_throttle_coef * self._convert_input(self.pitch, controller)
        msg.angular.z = self._ang_throttle_coef * self._convert_input(self.yaw, controller)
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

    def _set_lin_throttle(self, throttle_input):
        if abs(throttle_input) == 1 and self._last_lin_throttle_input == 0:
            self._lin_throttle_coef += throttle_input * self._lin_throttle_increment
            if self._lin_throttle_coef < 0:
                self._lin_throttle_coef = 0
        self._last_lin_throttle_input = throttle_input

    def _set_ang_throttle(self, throttle_input):
        if abs(throttle_input) == 1 and self._last_ang_throttle_input == 0:
            self._ang_throttle_coef += throttle_input * self._ang_throttle_increment
            if self._ang_throttle_coef < 0:
                self._ang_throttle_coef = 0
        self._last_ang_throttle_input = throttle_input

