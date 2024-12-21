import rclpy
from rclpy.node import Node

from ros_wrapper.service.unified_service import UnifiedService


class ROS2Service(UnifiedService):
    def __init__(self, node, name, service_class, execute_cb):
        """
                Initializes the ROS2Service with the given node, service name, service class, and handler.

                Args:
                    node (Node): The ROS 2 node instance.
                    name (str): The name of the service.
                    service_class (type): The type of the service.
                    execute_cb (callable): The handler function for the service.
                """
        self.__node = node
        self.__service = self.__node.create_service(service_class, name, execute_cb)

    def shutdown(self):
        """
                Shuts down the service.
                """
        self.__node.destroy_service(self.__service)
