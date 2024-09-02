import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def init_node(name, anonymous=True, verbose=False):
    if verbose:
        print("ROS2: Init Node")
    rclpy.init()
    global node
    node = Node(name)


def set_param(name, value):
    if node:
        node.declare_parameter(name, value)
    else:
        print("ROS2: ERROR: First init a node")

def get_param(param_name, default = None):
    return node.get_parameter_or(param_name, default)


def create_publisher(topic, msg_type, queue_size=10):
    print("ROS2: Creating Publisher")
    if node:
        return node.create_publisher(msg_type, topic, queue_size)
    else:
        print("ROS2: ERROR: First init a node")


def create_subscriber(topic, msg_type, callback):
    print("ROS2: Creating Subscriber")
    if node:
        node.create_subscription(msg_type, topic, callback, 10)
    else:
        print("ROS2: ERROR: First init a node")


def spin():
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()