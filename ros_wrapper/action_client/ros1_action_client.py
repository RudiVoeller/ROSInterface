import actionlib

class ROS1ActionClient:
    def __init__(self, action_name, action_type):
        self.client = actionlib.SimpleActionClient(action_name, action_type)
        self.action_type = action_type
        self.client.wait_for_server()

    def send_goal(self, goal):
        if self.action_type is None:
            print("Action type not set")
            return
       

        self.client.send_goal(goal)
        self.client.wait_for_result()


    def get_result(self):
        return self.client.get_result()