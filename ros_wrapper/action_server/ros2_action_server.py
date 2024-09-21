import rclpy
from rclpy.action import ActionServer

class ROS2ActionServer:
    def __init__(self, node, action_name, action_type, execute_callback):
        """
        Initialisiert den ROS 2 Action-Server.
        :param node: Der ROS 2 Node, der den Server hostet.
        :param action_name: Der Name der Action.
        :param action_type: Der Typ der Action-Nachricht (z.B. MyAction).
        :param execute_callback: Die Funktion, die aufgerufen wird, wenn ein Goal empfangen wird.
        """
        self.node = node
        self._user_callback = execute_callback  # Benutzerdefinierte execute_callback-Funktion
        self.server = ActionServer(
            node, action_type, action_name, execute_callback=self._execute_callback_wrapper
        )
        self.node.get_logger().info(f"ROS 2 Action Server '{action_name}' gestartet.")

    async def _execute_callback_wrapper(self, goal_handle):
        """Wrapper für die vom Benutzer bereitgestellte execute_callback."""
        await self._user_callback(self, goal_handle)  # Aufruf der benutzerdefinierten Callback-Funktion

    def publish_feedback(self, goal_handle, feedback):
        """
        Sendet Feedback an den Client.
        :param goal_handle: Das Handle des aktuellen Goals.
        :param feedback: Feedback-Nachricht.
        """
        goal_handle.publish_feedback(feedback)

    def set_succeeded(self, goal_handle, result):
        """
        Setzt das Ergebnis der Aktion als erfolgreich.
        :param goal_handle: Das Handle des aktuellen Goals.
        :param result: Ergebnis-Nachricht.
        """
        goal_handle.succeed()
        return result

    def set_aborted(self, goal_handle, result=None):
        """
        Markiert die Aktion als abgebrochen.
        :param goal_handle: Das Handle des aktuellen Goals.
        :param result: Optionales Ergebnis beim Abbruch.
        """
        goal_handle.abort()
        return result

    def is_preempt_requested(self, goal_handle):
        """
        Überprüft, ob die Aktion abgebrochen wurde (Preemption).
        :return: True, wenn eine Preemption angefordert wurde.
        """
        return not goal_handle.is_active

    def set_preempted(self, goal_handle):
        """
        Setzt die Aktion als abgebrochen durch den Benutzer (Preemption).
        """
        goal_handle.canceled()