from abc import ABC, abstractmethod


class UnifiedActionClient(ABC):
    """
     A unified client for interacting with both ROS 1 and ROS 2 action servers.

     Attributes:
         ros1_client (ROS1ActionClient): The ROS 1 action client instance.
         ros2_client (ROS2ActionClient): The ROS 2 action client instance.
         use_ros2 (bool): Flag to indicate whether to use ROS 2.
     """

    @abstractmethod
    def send_goal(self, goal, feedback_callback=None):
        """
                Sends a goal to the appropriate action server based on the ROS version.

                Args:
                    goal (Goal): The goal to send to the action server.
                """
        pass

    @abstractmethod
    def get_result(self):
        """
                Gets the result from the appropriate action server based on the ROS version.

                Returns:
                    Result: The result from the action server.
                """
        
        pass