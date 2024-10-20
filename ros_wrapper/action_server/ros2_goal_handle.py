from rclpy.action import ServerGoalHandle

class ROS2GoalHandle(GoalHandle):
    def __init__(self, goal_handle: ServerGoalHandle):
        super().__init__()
        self._goal_handle = goal_handle

    def succeed(self):
        super().succeed()
        self._goal_handle.succeed()

    def abort(self):
        super().abort()
        self._goal_handle.abort()

    def cancel(self):
        super().cancel()
        self._goal_handle.canceled()

    @property
    def request(self):
        return self._goal_handle.request

    @property
    def is_active(self):
        return self._goal_handle.is_active