from rclpy.node import Node

class ROS2Subscription:
    def __init__(self, node, topic, msg_type, callback):
        self.subscription = node.create_subscription(msg_type, topic, callback, 10)

    def unregister(self):
        self.subscription.destroy()