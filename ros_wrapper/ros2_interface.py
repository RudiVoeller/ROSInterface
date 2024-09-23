
import rclpy
from rclpy.node import Node

from ros_wrapper.param.unified_param import UnifiedParameter
from ros_wrapper.action_client.ros2_action_client import ROS2ActionClient
from ros_wrapper.action_server.ros2_action_server import ROS2ActionServer
from ros_wrapper.publisher.ros2_publisher import ROS2Publisher
from ros_wrapper.subscription.ros2_subscription import ROS2Subscription
from ros_wrapper.subscription.unified_subscription import UnifiedSubscription
from ros_wrapper.action_client.unified_action_client import UnifiedActionClient
from ros_wrapper.action_server.unified_action_server import UnifiedActionServer
from ros_wrapper.publisher.unified_publisher import UnifiedPublisher

node = None
action_server = None

def init_node(node_name, anonymous=False):
    rclpy.init()
    global node
    node = Node(node_name)


def create_action_server(action_name, action_type, execute_cb):
    if not node:
        print("ROS2: ERROR: First init a node")
        return None

    server = ROS2ActionServer(node, action_name, action_type, execute_cb)
    return UnifiedActionServer(server)

def create_action_client(action_name, action_type):
    if not node:
        print("ROS2: ERROR: First init a node")
        return None

    client = ROS2ActionClient(node, action_name, action_type)
    return UnifiedActionClient(client)

def set_param(name, value):
    if node:
        node.declare_parameter(name, value)
    else:
        print("ROS2: ERROR: First init a node")

def get_param(param_name, default = None):
    if not node:
        print("ROS2: ERROR: First init a node")
        return None
    value = node.get_parameter_or(param_name, default)
    return UnifiedParameter(value)
def subscription_count_per_topic(topic_name):
    if not node:
        print("ROS2: ERROR: First init a node")
        return None
    return node.count_subscribers(topic_name)

def publisher_count_per_topic(topic_name):
    if not node:
        print("ROS2: ERROR: First init a node")
        return None
    return node.count_publishers(topic_name)

def create_publisher(topic, msg_type):
    print("ROS2: Creating Publisher")
    if node is not None:
        publisher = ROS2Publisher(node, topic, msg_type)
        return UnifiedPublisher(publisher)
    else:
        print("ROS2: ERROR: First init a node")


def create_subscriber(topic, msg_type, callback):
    print("ROS2: Creating Subscriber")
    if node:
        subscription = ROS2Subscription(node, topic, msg_type, callback)
        return UnifiedSubscription(subscription)
    else:
        print("ROS2: ERROR: First init a node")

def create_service(name,service_class, handler):
    if node:
        node.create_service(service_class, name, handler)
    else:
        print("ROS2: ERROR: First init a node")

def call_service(service_name, service_type, *args):
    if not node:
        print("ROS2: ERROR: First init a node")
        return None
    client = node.create_client(service_type, service_name)

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f"Service '{service_name}' not available, waiting...")

    request = service_type.Request()
    for arg, value in enumerate(args):
        setattr(request, f'arg{arg}', value)

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        return future.result()
    else:
        node.get_logger().error('Service call failed.')
        return None

def get_all_nodes():
    if node:
        node_names_and_namespaces = node.get_node_names_and_namespaces()
        return [name for name, namespace in node_names_and_namespaces]
    else:
        print("ROS2: ERROR: First init a node")
        return []

def get_all_services():
    if node:
        return node.get_service_names_and_types()
    else:
        print("ROS2: ERROR: First init a node")
        return []
def get_all_topics():
    if node:
        return node.get_topic_names_and_types()
    else:
        print("ROS2: ERROR: First init a node")
        return []
def spin():
    if node:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

def spin_once():
    if node:
        rclpy.spin_once(node)