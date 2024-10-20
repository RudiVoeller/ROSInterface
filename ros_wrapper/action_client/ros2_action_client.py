from rclpy.action import ActionClient
import rclpy
class ROS2ActionClient:
    action_type = None
    def __init__(self, node, action_name, action_type):
        self.client = ActionClient(node, action_type, action_name)
        self.action_type = action_type
        while not self.client.wait_for_server(timeout_sec=1.0):
            node.get_logger().info(f"Action-Client '{action_name}' in ROS 2 wartet auf den Server.")

    def send_goal(self, goal):
        if self.action_type is None:
            print("Action type not set")
            return
        goal_msg = self.action_type.Goal()
        goal_msg.order = goal

        self.future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.client._node, self.future)
        self.goal_handle = self.future.result()

    def wait_for_result(self):
        if self.goal_handle.accepted:
            self.result_future = self.goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.client._node, self.result_future)

    def get_result(self):
        return self.result_future.result().result if self.goal_handle.accepted else None