import rospy

from ros_wrapper.subscription.unified_subscription import UnifiedSubscription


class ROS1Publisher(UnifiedSubscription):
    def __init__(self, topic, msg_type, queue_size=10):
        """
                Initializes the ROS1Publisher with the given topic, message type, and queue size.

                Args:
                    topic (str): The topic to publish to.
                    msg_type (type): The type of the message to publish.
                    queue_size (int, optional): The size of the message queue. Defaults to 10.
                """
        self.__publisher = rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def unregister(self):
        """
                Unregisters the publisher from the ROS master.
                """
        self.__publisher.unregister()

    def publish(self, msg):
        """
                Publishes a message to the topic.

                Args:
                    msg (msg_type): The message to publish.
                """
        self.__publisher.publish(msg)