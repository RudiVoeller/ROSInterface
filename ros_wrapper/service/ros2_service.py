import rclpy
from rclpy.node import Node

class ROS2Service:
    def __init__(self, node, name, service_class, execute_cb):
        """Initializes the ROS2Service.

        Args:
            node (Node): The ROS2 node.
            name (str): The name of the service.
            service_class (Service): The class of the ROS service.
            handler (function): The callback function for the service.
        """
        self.node = node
        self.service = self.node.create_service(service_class, name, execute_cb)

    def shutdown(self):
        """Shuts down the service."""

        self.node.destroy_service(self.service)
