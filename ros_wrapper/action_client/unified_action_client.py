class UnifiedActionClient:
    """
     A unified client for interacting with both ROS 1 and ROS 2 action servers.

     Attributes:
         ros1_client (ROS1ActionClient): The ROS 1 action client instance.
         ros2_client (ROS2ActionClient): The ROS 2 action client instance.
         use_ros2 (bool): Flag to indicate whether to use ROS 2.
     """

    def __init__(self, client):
        """
                Initializes the UnifiedActionClient with the given ROS 1 and ROS 2 clients.

                Args:
                    ros1_client (ROS1ActionClient, optional): The ROS 1 action client instance.
                    ros2_client (ROS2ActionClient, optional): The ROS 2 action client instance.
                    use_ros2 (bool, optional): Flag to indicate whether to use ROS 2. Defaults to False.
                """
        self.client = client

    def send_goal(self, goal):
        """
                Sends a goal to the appropriate action server based on the ROS version.

                Args:
                    goal (Goal): The goal to send to the action server.
                """
        self.client.send_goal(goal)


    def get_result(self):
        """
                Gets the result from the appropriate action server based on the ROS version.

                Returns:
                    Result: The result from the action server.
                """
        
        return self.client.get_result()