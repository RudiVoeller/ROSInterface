from rclpy.action import ActionClient
import rclpy
class ROS2ActionClient:
    """
      A client for interacting with ROS 2 action servers.

      Attributes:
          client (ActionClient): The action client instance.
          action_type (type): The type of the action.
      """
    action_type = None
    def __init__(self, node, action_name, action_type):

        """
                Initializes the ROS2ActionClient with the given node, action name, and type.

                Args:
                    node (Node): The ROS 2 node.
                    action_name (str): The name of the action.
                    action_type (type): The type of the action.
                """

        self.client = ActionClient(node, action_type, action_name)
        self.action_type = action_type
        while not self.client.wait_for_server(timeout_sec=1.0):
            node.get_logger().info(f"Action-Client '{action_name}' in ROS 2 wartet auf den Server.")

    def send_goal(self, goal):

        """
                Sends a goal to the action server.

                Args:
                    goal (Goal): The goal to send to the action server.
                """

        if self.action_type is None:
            print("Action type not set")
            return
        goal_msg = self.action_type.Goal()
        goal_msg.order = goal

        self.future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.client._node, self.future)
        self.goal_handle = self.future.result()


    def get_result(self):

        """
                Gets the result from the action server.

                Returns:
                    Result: The result from the action server.
                """

        return self.result_future.result().result if self.goal_handle.accepted else None