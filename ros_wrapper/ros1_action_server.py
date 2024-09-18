import rospy
import actionlib

class ROS1ActionServer:
    def __init__(self, action_name, action_type, execute_callback):
        """
        Initialisiert den ROS 1 Action-Server.
        :param action_name: Der Name der Action.
        :param action_type: Der Typ der Action-Nachricht (z.B. MyAction).
        :param execute_callback: Die Funktion, die aufgerufen wird, wenn ein Goal empfangen wird.
        """
        self.server = actionlib.SimpleActionServer(
            action_name, action_type, execute_cb=self._execute_callback_wrapper, auto_start=False
        )
        self._user_callback = execute_callback  # Benutzerdefinierte execute_callback-Funktion
        self.server.start()
        rospy.loginfo(f"ROS 1 Action Server '{action_name}' gestartet.")

    def _execute_callback_wrapper(self, goal):
        """Wrapper für die vom Benutzer bereitgestellte execute_callback."""
        self._user_callback(self, goal)  # Aufruf der benutzerdefinierten Callback-Funktion

    def publish_feedback(self, feedback):
        """
        Sendet Feedback an den Client.
        :param feedback: Feedback-Nachricht.
        """
        self.server.publish_feedback(feedback)

    def set_succeeded(self, result):
        """
        Setzt das Ergebnis der Aktion als erfolgreich.
        :param result: Ergebnis-Nachricht.
        """
        self.server.set_succeeded(result)

    def set_aborted(self, result=None):
        """
        Markiert die Aktion als abgebrochen.
        :param result: Optionales Ergebnis beim Abbruch.
        """
        self.server.set_aborted(result)

    def is_preempt_requested(self):
        """
        Überprüft, ob die Aktion abgebrochen wurde (Preemption).
        :return: True, wenn eine Preemption angefordert wurde.
        """
        return self.server.is_preempt_requested()

    def set_preempted(self):
        """
        Setzt die Aktion als abgebrochen durch den Benutzer (Preemption).
        """
        self.server.set_preempted()