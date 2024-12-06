import rospy

class ROS1Publisher:
    def __init__(self, topic, msg_type, queue_size=10):
        """Initializes the ROS1Publisher.

             Args:
                 topic (str): The name of the topic to publish to.
                 msg_type (rospy.Message): The type of the ROS message.
                 queue_size (int, optional): The size of the message queue. Defaults to 10.
             """
        self.publisher = rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def unregister(self):
        """Unregisters the publisher to stop it from publishing messages."""

        self.publisher.unregister()

    def publish(self, msg):
        """Publishes a message to the topic.

                Args:
                    msg (rospy.Message): The message to publish.
                """
        self.publisher.publish(msg)