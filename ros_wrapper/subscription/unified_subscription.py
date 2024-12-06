# ros_wrapper/unified_subscription.py
class UnifiedSubscription:
    def __init__(self, subscription):
        """Initializes the UnifiedSubscription.

                Args:
                    subscription (Subscription): The ROS subscription (either ROS1 or ROS2).
                """
        self.subscription = subscription

    def unregister(self):
        """Unregisters the subscription to stop receiving messages."""

        self.subscription.unregister()