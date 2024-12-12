from rclpy.node import Node

class ROS2Subscription:
    def __init__(self, node, topic, msg_type, callback):
        """
                Initializes the ROS2Subscription with the given node, topic, message type, callback, and QoS profile.

                Args:
                    node (Node): The ROS 2 node instance.
                    topic (str): The topic to subscribe to.
                    msg_type (type): The type of the message to subscribe to.
                    callback (callable): The callback function to handle incoming messages.
                    qos_profile (int, optional): The QoS profile for the subscription. Defaults to 10.
                """
        self.subscription = node.create_subscription(msg_type, topic, callback, 10)

    def unregister(self):
        """
                Unregisters the subscriber from the ROS 2 node.
                """
        self.subscription.destroy()