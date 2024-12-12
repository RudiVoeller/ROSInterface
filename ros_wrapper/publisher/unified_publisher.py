class UnifiedPublisher:
    def __init__(self, publisher):
        """
                Publishes a message to the topic.

                Args:
                    msg (msg_type): The message to publish.
                """
        self.publisher = publisher

    def publish(self, msg):
        """
                Publishes a message to the appropriate topic based on the ROS version.

                Args:
                    msg (msg_type): The message to publish.
                """
        self.publisher.publish(msg)

    def unregister(self):
        """
                Unregisters the publisher from the ROS master or ROS 2 node.
                """
        self.publisher.unregister()