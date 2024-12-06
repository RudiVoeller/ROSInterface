import rospy

class ROS1Subscription:
    def __init__(self, topic, msg_type, callback):
        """Initializes the ROS1Subscription.

                Args:
                    topic (str): The name of the topic to subscribe to.
                    msg_type (Message): The type of the ROS message.
                    callback (function): The callback function for the subscription.
                """
        self.subscription = rospy.Subscriber(topic, msg_type, callback)

    def unregister(self):
        """Unregisters the subscription to stop receiving messages."""

        self.subscription.unregister()