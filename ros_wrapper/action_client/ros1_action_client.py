import actionlib

class ROS1ActionClient:
    def __init__(self, action_name, action_type):
        self.client = actionlib.SimpleActionClient(action_name, action_type)
        self.client.wait_for_server()

    def send_goal(self, goal):
        self.client.send_goal(goal)

    def wait_for_result(self):
        self.client.wait_for_result()

    def get_result(self):
        return self.client.get_result()