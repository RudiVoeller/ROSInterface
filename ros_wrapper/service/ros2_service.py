import rclpy
from rclpy.node import Node

class ROS2Service:
    def __init__(self, node, name, service_class, execute_cb):
        """
                Initializes the ROS2Service with the given node, service name, service class, and handler.

                Args:
                    node (Node): The ROS 2 node instance.
                    name (str): The name of the service.
                    service_class (type): The type of the service.
                    handler (callable): The handler function for the service.
                """
        self.node = node
        self.service = self.node.create_service(service_class, name, execute_cb)

    def shutdown(self):
        """
                Shuts down the service.
                """
        self.node.destroy_service(self.service)
