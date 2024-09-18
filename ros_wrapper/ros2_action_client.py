from rclpy.action import ActionClient

class ROS2ActionClient:
    def __init__(self, node, action_name, action_type):
        self.client = ActionClient(node, action_type, action_name)
        while not self.client.wait_for_server(timeout_sec=1.0):
            node.get_logger().info(f"Action-Client '{action_name}' in ROS 2 wartet auf den Server.")

    def send_goal(self, goal):
        self.future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.client._node, self.future)
        self.goal_handle = self.future.result()

    def wait_for_result(self):
        if self.goal_handle.accepted:
            self.result_future = self.goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.client._node, self.result_future)

    def get_result(self):
        return self.result_future.result().result if self.goal_handle.accepted else None