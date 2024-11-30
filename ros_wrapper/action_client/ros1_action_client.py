import actionlib

class ROS1ActionClient:
    """
      A client for interacting with ROS 1 action servers.

      Attributes:
          client (SimpleActionClient): The action client instance.
          action_type (type): The type of the action.
      """

    def __init__(self, action_name, action_type):
        """
                Initializes the ROS1ActionClient with the given action name and type.

                Args:
                    action_name (str): The name of the action.
                    action_type (type): The type of the action.
                """

        self.client = actionlib.SimpleActionClient(action_name, action_type)
        self.action_type = action_type
        self.client.wait_for_server()

    def send_goal(self, goal):
        """
                Sends a goal to the action server.

                Args:
                    goal (Goal): The goal to send to the action server.
                """

        if self.action_type is None:
            print("Action type not set")
            return
       

        self.client.send_goal(goal)
        self.client.wait_for_result()


    def get_result(self):
        """
                Gets the result from the action server.

                Returns:
                    Result: The result from the action server.
                """

        return self.client.get_result()