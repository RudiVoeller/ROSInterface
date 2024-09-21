# ros_wrapper/unified_subscription.py
class UnifiedSubscription:
    def __init__(self, subscription):
        self.subscription = subscription

    def unregister(self):
        self.subscription.unregister()