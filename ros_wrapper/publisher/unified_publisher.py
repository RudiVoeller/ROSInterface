class UnifiedPublisher:
    def __init__(self, publisher):
        """Initializes the UnifiedPublisher.

               Args:
                   publisher (Publisher): The ROS publisher (either ROS1 or ROS2).
               """
        self.publisher = publisher

    def publish(self, msg):
        """Publishes a message to the topic.

                Args:
                    msg (Message): The message to publish.
                """
        self.publisher.publish(msg)

    def unregister(self):
        """Unregisters the publisher to stop it from publishing messages."""

        self.publisher.unregister()