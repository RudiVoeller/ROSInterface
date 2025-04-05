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
        self._node = node
        self._goal_handle = None
        self._user_callback = execute_callback  # Benutzerdefinierte execute_callback-Funktion

        if  inspect.iscoroutinefunction(execute_callback) is not True:
            self.server = ActionServer(
                node, action_type, action_name, execute_callback=self._execute_callback_wrapper
            )
        else:
            self.server = ActionServer(
                node, action_type, action_name, execute_callback=self._execute_callback_wrapper_async
            )


    def _execute_callback_wrapper(self, goal_handle):
        self._goal_handle = goal_handle
        return self._user_callback(self, goal_handle)

    async def _execute_callback_wrapper_async(self, goal_handle):
        self._goal_handle = goal_handle
        result = await self._user_callback(self, goal_handle)
        return result

    def publish_feedback(self, feedback):
        """
          Sends feedback to client.

          feedback: Feedback-Message.
        """
        if self._goal_handle:
            self._goal_handle.publish_feedback(feedback)

    def set_succeeded(self, result):
        """
              Sets the action server state to succeeded.

              Args:
                  goal_handle (GoalHandle): The goal handle.
                  result (Result): The result to send to the client.
              """
        if self._goal_handle:
            self._goal_handle.succeed()
            return result

    def set_aborted(self):
        """
              Sets the action server state to aborted.

              Args:
                  goal_handle (GoalHandle): The goal handle.
                  result (Result): The result to send to the client.
              """
        if self._goal_handle:
            self._goal_handle.abort()

    def is_preempt_requested(self):
        """
              Checks if the action has been preempted.

              Returns:
                  True if a preemption has been requested.
                  """
        if self._goal_handle:
            return not self._goal_handle.is_active
        return False

    def set_preempted(self):
        """
           Sets the action server state to preempted.

           Args:
               goal_handle (GoalHandle): The goal handle.
           """
        if self._goal_handle:
            self._goal_handle.canceled()
