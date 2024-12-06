from rclpy.node import Node

class ROS2Subscription:
    def __init__(self, node, topic, msg_type, callback):
        """Initializes the ROS2Subscription.

                Args:
                    node (Node): The ROS2 node.
                    topic (str): The name of the topic to subscribe to.
                    msg_type (Message): The type of the ROS message.
                    callback (function): The callback function for the subscription.
                """
        self.subscription = node.create_subscription(msg_type, topic, callback, 10)

    def unregister(self):
        """Unregisters the subscription to stop receiving messages."""

        self.subscription.destroy()