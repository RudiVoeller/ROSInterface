from abc import ABC, abstractmethod

class UnifiedSubscriber(ABC):

    @abstractmethod
    def unregister(self):
        """
                Unregisters the subscriber from the ROS master or ROS 2 node.
                """
        pass