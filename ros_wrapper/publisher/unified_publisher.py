from abc import ABC, abstractmethod


class UnifiedPublisher(ABC):

    @abstractmethod
    def publish(self, msg):
        """
                Publishes a message to the appropriate topic based on the ROS version.

                Args:
                    msg (msg_type): The message to publish.
                """
        pass
    @abstractmethod
    def unregister(self):
        """
                Unregisters the publisher from the ROS master or ROS 2 node.
                """
        pass