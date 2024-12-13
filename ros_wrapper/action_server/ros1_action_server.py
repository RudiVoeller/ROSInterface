import rospy
import actionlib

from ros_wrapper.action_server.unified_action_server import UnifiedActionServer


class ROS1ActionServer(UnifiedActionServer):
    """
             A server for handling ROS 1 action requests.

             Attributes:
                 __server (SimpleActionServer): The action server instance.
                 action_type (type): The type of the action.
             """
    def __init__(self, action_name, action_type, execute_callback):


        """
               Initializes the ROS1ActionServer with the given action name, type, and callback.

               Args:
                   action_name (str): The name of the action.
                   action_type (type): The type of the action.
                   execute_cb (function): The callback function to execute when a goal is received.
               """
        self.__user_callback = execute_callback  # Benutzerdefinierte execute_callback-Funktion
        self.__server = actionlib.SimpleActionServer(
            action_name, action_type, execute_cb=self._execute_callback_wrapper, auto_start=False
        )
        self.__server.start()

    def _execute_callback_wrapper(self, goal):
        """Wrapper for the user provided execute_callback."""
        return self.__user_callback(self, goal)  # Aufruf der benutzerdefinierten Callback-Funktion

    def publish_feedback(self, feedback):
        """
         Sends feedback to client.

         feedback: Feedback-Message.
        """
        self.__server.publish_feedback(feedback)

    def set_succeeded(self, result):
        """
              Sets the action server state to succeeded.

              Args:
                  result (Result): The result to send to the client.
              """
        self.__server.set_succeeded(result)

    def set_aborted(self, result=None):
        """
             Sets the action server state to aborted.

             Args:
                 result (Result): The result to send to the client.
             """
        self.__server.set_aborted(result)

    def is_preempt_requested(self):
        """
             Checks if the action has been preempted.

             Returns:
                 True if a preemption has been requested.
             """
        return self.__server.is_preempt_requested()

    def set_preempted(self):
        """
             Sets the action server state to preempted.
             """
        self.__server.set_preempted()