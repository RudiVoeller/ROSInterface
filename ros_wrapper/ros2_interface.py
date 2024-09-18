import asyncio

import rclpy
from rclpy.node import Node

from ros_wrapper.ros2_action_client import ROS2ActionClient
from ros_wrapper.ros2_action_server import ROS2ActionServer
from ros_wrapper.unified_action_client import UnifiedActionClient
from ros_wrapper.unified_action_server import UnifiedActionServer
from ros_wrapper.unified_publisher import UnifiedPublisher

global node
global action_server

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
    return node.get_parameter_or(param_name, default)

def subscription_count_per_topic(topic_name):
    return node.count_subscribers(topic_name)

def publisher_count_per_topic(topic_name):
    return node.count_publishers(topic_name)

def create_publisher(topic, msg_type, queue_size=10):
    print("ROS2: Creating Publisher")
    if node:
        publisher = node.create_publisher(msg_type, topic, queue_size)
        return UnifiedPublisher(publisher)
    else:
        print("ROS2: ERROR: First init a node")


def create_subscriber(topic, msg_type, callback):
    print("ROS2: Creating Subscriber")
    if node:
        node.create_subscription(msg_type, topic, callback, 10)
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