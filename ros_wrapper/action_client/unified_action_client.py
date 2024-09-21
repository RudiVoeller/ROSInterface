class UnifiedActionClient:
    def __init__(self, client):
        self.client = client

    def send_goal(self, goal):
        self.client.send_goal(goal)

    def wait_for_result(self):
        self.client.wait_for_result()

    def get_result(self):
        return self.client.get_result()