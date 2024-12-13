import actionlib

from ros_wrapper.action_client.unified_action_client import UnifiedActionClient


class ROS1ActionClient(UnifiedActionClient):
    """
      A client for interacting with ROS 1 action servers.

      Attributes:
          __client (SimpleActionClient): The action client instance.
          __action_type (type): The type of the action.
      """

    def __init__(self, action_name, action_type):
        """
                Initializes the ROS1ActionClient with the given action name and type.

                Args:
                    action_name (str): The name of the action.
                    action_type (type): The type of the action.
                """

        self.__client = actionlib.SimpleActionClient(action_name, action_type)
        self.__action_type = action_type
        self.__client.wait_for_server()

    def send_goal(self, goal):
        """
                Sends a goal to the action server.

                Args:
                    goal (Goal): The goal to send to the action server.
                """

        if self.__action_type is None:
            print("Action type not set")
            return
       

        self.__client.send_goal(goal)
        self.__client.wait_for_result()


    def get_result(self):
        """
                Gets the result from the action server.

                Returns:
                    Result: The result from the action server.
                """

        return self.__client.get_result()