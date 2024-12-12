# ros_wrapper/unified_subscription.py
class UnifiedSubscription:
    def __init__(self, subscription):
        """
                Initializes the UnifiedSubscription with the given ROS 1 or ROS 2 subscription.

                Args:
                    subscription (ROS1Subscription or ROS2Subscription): The underlying subscription instance.
                """
        self.subscription = subscription

    def unregister(self):
        """
                Unregisters the subscriber from the ROS master or ROS 2 node.
                """
        self.subscription.unregister()