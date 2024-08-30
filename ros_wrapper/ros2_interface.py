import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def init_node(name, anonymous=True):
    rclpy.init()
    global node
    node = Node(name)

def create_publisher(topic, msg_type, queue_size=10):
    return node.create_publisher(msg_type, topic, queue_size)

def create_subscriber(topic, msg_type, callback):
    node.create_subscription(msg_type, topic, callback, 10)

def spin():
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()