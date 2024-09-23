from rclpy.node import Node
from rclpy.qos import QoSProfile


class ROS2Publisher:
    def __init__(self, node, topic, msg_type, qos_profile=QoSProfile(depth=10)):
        self.publisher = node.create_publisher(msg_type, topic, qos_profile)

    def unregister(self):
        self.publisher.destroy()

    def publish(self, msg):
        self.publisher.publish(msg)