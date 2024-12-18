import rospy

from ros_wrapper.subscription.unified_subscription import UnifiedSubscription


class ROS1Subscription(UnifiedSubscription):
    def __init__(self, topic, msg_type, callback):
        """
                Initializes the ROS1Subscription with the given topic, message type, callback, and queue size.

                Args:
                    topic (str): The topic to subscribe to.
                    msg_type (type): The type of the message to subscribe to.
                    callback (callable): The callback function to handle incoming messages.
                    queue_size (int, optional): The size of the message queue. Defaults to 10.
                """
        self.__subscription = rospy.Subscriber(topic, msg_type, callback)

    def unregister(self):
        """
                Unregisters the subscriber from the ROS master.
                """
        self.__subscription.unregister()