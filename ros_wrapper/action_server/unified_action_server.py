class UnifiedActionServer:
    """
        A unified server for handling both ROS 1 and ROS 2 action requests.

        Attributes:
            ros1_server (ROS1ActionServer): The ROS 1 action server instance.
            ros2_server (ROS2ActionServer): The ROS 2 action server instance.
            use_ros2 (bool): Flag to indicate whether to use ROS 2.
        """

    def __init__(self, server):
        """
                Initializes the UnifiedActionServer with the given ROS 1 and ROS 2 servers.

                Args:
                    ros1_server (ROS1ActionServer, optional): The ROS 1 action server instance.
                    ros2_server (ROS2ActionServer, optional): The ROS 2 action server instance.
                    use_ros2 (bool, optional): Flag to indicate whether to use ROS 2. Defaults to False.
                """
        self.server = server

    def publish_feedback(self, feedback):
        self.server.publish_feedback(feedback)

    def set_succeeded(self, result):
        """
                Sets the action server state to succeeded.

                Args:
                    result (Result): The result to send to the client.
                """

        self.server.set_succeeded(result)

    def set_aborted(self, result=None):
        """
                Sets the action server state to aborted.

                Args:
                    result (Result): The result to send to the client.
                """
        self.server.set_aborted(result)

    def is_preempt_requested(self):
        return self.server.is_preempt_requested()

    def set_preempted(self):
        """
               Sets the action server state to preempted.
               """
        self.server.set_preempted()