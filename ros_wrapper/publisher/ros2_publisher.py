from rclpy.node import Node
from rclpy.qos import QoSProfile


class ROS2Publisher:
    def __init__(self, node, topic, msg_type, qos_profile=QoSProfile(depth=10)):
        """Publishes a message to the topic.

                Args:
                    msg (rospy.Message): The message to publish.
                """
        self.publisher = node.create_publisher(msg_type, topic, qos_profile)

    def unregister(self):
        """Unregisters the publisher to stop it from publishing messages."""

        self.publisher.destroy()

    def publish(self, msg):
        """Publishes a message to the topic.

             Args:
                 msg (rclpy.Message): The message to publish.
             """
        self.publisher.publish(msg)