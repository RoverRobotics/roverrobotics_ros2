#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import yaml
from modules.Controller import Controller, Axis, Button
from modules.Topics import Topics, TwistTopic

class Mapper(Node):
    def __init__(self, joy_topic, name="joy_manager"):
        super().__init__(name)
        self._name = name
        self._joy_topic = joy_topic
        self._publish_topics = []

        self._joy_subscriber = self.create_subscription(Joy, joy_topic, self._joy_callback, 10)
        controller = self.declare_parameter("controller", "NULL").value
        topics = self.declare_parameter("topics", "NULL").value
        self._controller = self._configure_controller_mapping(open_yaml(controller)) if controller else None
        self._topics = self._register_topics(open_yaml(topics)) if topics else None

    def _joy_callback(self, msg: Joy):
        if not self._controller:
            self.get_logger().fatal('Axis and Button mappings must be defined.')
            rclpy.shutdown()
        elif not self._topics:
            self.get_logger().fatal('No topics specified')
            rclpy.shutdown()
        self._controller.update_states(**{'axes': msg.axes, 'buttons': msg.buttons})

        self._topics.publish(self._controller)

    def _configure_controller_mapping(self, button_mappings: dict):
        return Controller(button_mappings)

    def _register_topics(self, topics: dict):
        return Topics(self, topics)

    def update_topics(self):
        for topic in self._topics:
            topic.publish_topic(self.state)


def open_yaml(file_name):
    with open(file_name) as stream:
        return yaml.safe_load(stream)

def main(args=None):
    rclpy.init(args=args)
    mapper = Mapper('/joy')
    rclpy.spin(mapper)

if __name__ == '__main__':
    main()
