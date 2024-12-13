from abc import ABC, abstractmethod


class UnifiedActionServer(ABC):
    """
        A unified server for handling both ROS 1 and ROS 2 action requests.

        Attributes:
            ros1_server (ROS1ActionServer): The ROS 1 action server instance.
            ros2_server (ROS2ActionServer): The ROS 2 action server instance.
            use_ros2 (bool): Flag to indicate whether to use ROS 2.
        """

    @abstractmethod
    def publish_feedback(self, feedback):
        pass

    @abstractmethod
    def set_succeeded(self, result):
        """
                Sets the action server state to succeeded.

                Args:
                    result (Result): The result to send to the client.
                """
        pass

    @abstractmethod
    def set_aborted(self, result=None):
        """
                Sets the action server state to aborted.

                Args:
                    result (Result): The result to send to the client.
                """
        pass

    @abstractmethod
    def is_preempt_requested(self):
        pass

    @abstractmethod
    def set_preempted(self):
        """
               Sets the action server state to preempted.
               """
        pass