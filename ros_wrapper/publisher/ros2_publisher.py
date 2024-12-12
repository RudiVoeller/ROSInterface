from rclpy.node import Node
from rclpy.qos import QoSProfile


class ROS2Publisher:
    def __init__(self, node, topic, msg_type, qos_profile=QoSProfile(depth=10)):
        """
                Publishes a message to the topic.

                Args:
                    msg (msg_type): The message to publish.
                """
        self.publisher = node.create_publisher(msg_type, topic, qos_profile)

    def unregister(self):
        """
                Unregisters the publisher from the ROS 2 node.
                """
        self.publisher.destroy()

    def publish(self, msg):
        """
                Publishes a message to the topic.

                Args:
                    msg (msg_type): The message to publish.
                """
        self.publisher.publish(msg)