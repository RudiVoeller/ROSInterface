from rclpy.action import ActionClient
import rclpy

from ros_wrapper.action_client.unified_action_client import UnifiedActionClient


class ROS2ActionClient(UnifiedActionClient):
    """
      A client for interacting with ROS 2 action servers.

      Attributes:
          __client (ActionClient): The action client instance.
          __action_type (type): The type of the action.
      """
    __action_type = None
    def __init__(self, node, action_name, action_type):

        """
                Initializes the ROS2ActionClient with the given node, action name, and type.

                Args:
                    node (Node): The ROS 2 node.
                    action_name (str): The name of the action.
                    action_type (type): The type of the action.
                """

        self.__goal_handle = None
        self.__future = None
        self.__client = ActionClient(node, action_type, action_name)
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
        goal_msg = self.__action_type.Goal()
        goal_msg.order = goal

        self.__future = self.__client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.__client._node, self.__future)
        self.__goal_handle = self.__future.result()


    def get_result(self):

        """
                Gets the result from the action server.

                Returns:
                    Result: The result from the action server.
                """

        return self.__future.result().result if self.__goal_handle.accepted else None