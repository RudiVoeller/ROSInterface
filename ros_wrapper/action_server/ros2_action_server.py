import inspect
from rclpy.action import ActionServer

from ros_wrapper.action_server.unified_action_server import UnifiedActionServer


class ROS2ActionServer(UnifiedActionServer):
    """
        A server for handling ROS 2 action requests.

        Attributes:
            server (ActionServer): The action server instance.
            action_type (type): The type of the action.
        """

    def __init__(self, node, action_name, action_type, execute_callback):
        """
              Initializes the ROS2ActionServer with the given node, action name, type, and callback.

              Args:
                  node (Node): The ROS 2 node.
                  action_name (str): The name of the action.
                  action_type (type): The type of the action.
                  execute_cb (function): The callback function to execute when a goal is received.
              """
        self.node = node
        self._user_callback = execute_callback  # Benutzerdefinierte execute_callback-Funktion

        if  inspect.iscoroutinefunction(execute_callback) is not True:
            self.server = ActionServer(
                node, action_type, action_name, execute_callback=self._execute_callback_wrapper
            )
        else:
            self.server = ActionServer(
                node, action_type, action_name, execute_callback=self._execute_callback_wrapper_async
            )


        self.node.get_logger().info(f"ROS 2 Action Server '{action_name}' gestartet.")

    def _execute_callback_wrapper(self, goal_handle):
        return self._user_callback(self, goal_handle)

    async def _execute_callback_wrapper_async(self, goal_handle):
        result = await self._user_callback(self, goal_handle)
        return result
# TODO: Goal Handle in Konstruktor mit aufnehmen.
    def publish_feedback(self, goal_handle, feedback):
        """
          Sends feedback to client.

          feedback: Feedback-Message.
        """
        goal_handle.publish_feedback(feedback)

    def set_succeeded(self, goal_handle, result):
        """
              Sets the action server state to succeeded.

              Args:
                  goal_handle (GoalHandle): The goal handle.
                  result (Result): The result to send to the client.
              """
        goal_handle.succeed()
        return result

    def set_aborted(self, goal_handle):
        """
              Sets the action server state to aborted.

              Args:
                  goal_handle (GoalHandle): The goal handle.
                  result (Result): The result to send to the client.
              """
        goal_handle.abort()

    def is_preempt_requested(self, goal_handle):
        """
              Checks if the action has been preempted.

              Returns:
                  True if a preemption has been requested.
                  """
        return not goal_handle.is_active

    def set_preempted(self, goal_handle):
        """
           Sets the action server state to preempted.

           Args:
               goal_handle (GoalHandle): The goal handle.
           """
        goal_handle.canceled()