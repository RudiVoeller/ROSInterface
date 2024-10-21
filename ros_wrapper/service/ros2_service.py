import rclpy
from rclpy.node import Node

class ROS2Service:
    def __init__(self, node, name, service_class, handler):
        self.node = node
        self.service = self.node.create_service(service_class, name, handler)

    def shutdown(self):
        self.node.destroy_service(self.service)
        self.node.destroy_node()