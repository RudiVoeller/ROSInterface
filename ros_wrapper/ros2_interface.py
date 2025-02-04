import random
import string

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from ros_wrapper.param.unified_param import UnifiedParameter
from ros_wrapper.action_client.ros2_action_client import ROS2ActionClient
from ros_wrapper.action_server.ros2_action_server import ROS2ActionServer
from ros_wrapper.publisher.ros2_publisher import ROS2Publisher
from ros_wrapper.service.ros2_service import ROS2Service
from ros_wrapper.subscription.ros2_subscriber import ROS2Subscriber


_node = None

def __is_node_initiialized(a_func):

    def wrapTheFunction(*args, **kwargs):
        global _node
        if not _node:
            print("ROS2: WARNING: No node initialized, Init node")
            letters = string.ascii_letters + string.digits
            name = ''.join(random.choice(letters) for i in range(10))
            rclpy.init()
            _node = Node(name)

        return a_func(*args, **kwargs)


    return wrapTheFunction


def init_node(node_name, anonymous=False):
    """
    Creates a ROS 2 node.

    Args:
        node_name (str): Name of the node.
        anonymous (bool, optional): If True, the node name will be made unique by adding random characters. Defaults to False.
    """
    rclpy.init()
    global _node
    _node = Node(node_name)

@__is_node_initiialized
def create_action_server(action_name, action_type, execute_cb):
    """
    Creates an action server and returns it as a UnifiedActionServer.

    Args:
        action_name (str): Name of the action.
        action_type (type): Type of the action.
        execute_cb (callable): Callback function to execute the action.

    Returns:
        UnifiedActionServer: The created action server.
    """


    server = ROS2ActionServer(_node, action_name, action_type, execute_cb)
    return server

@__is_node_initiialized
def create_action_client(action_name, action_type):
    """
    Creates an action client and returns it as a UnifiedActionClient.

    Args:
        action_name (str): Name of the action.
        action_type (type): Type of the action.

    Returns:
        UnifiedActionClient: The created action client.
    """

    client = ROS2ActionClient(_node, action_name, action_type)
    return client

@__is_node_initiialized
def set_param(name, value):
    """
    Sets a parameter on the node.

    Args:
        name (str): Name of the parameter.
        value: Value of the parameter.
    """
    _node.declare_parameter(name, value)

@__is_node_initiialized
def get_param(param_name, default = None):
    """
    Gets a parameter from any node that has it.

    Args:
        param_name (str): Name of the parameter.
        default: Default value if the parameter is not found. Defaults to None.

    Returns:
        UnifiedParameter: The parameter value.
    """

    try:
        node_names_and_namespaces = _node.get_node_names_and_namespaces()
        for node_name, namespace in node_names_and_namespaces:
            client = _node.create_client(GetParameters, f'{namespace}/{node_name}/get_parameters')
            request = GetParameters.Request()
            request.names = [param_name]

            future = client.call_async(request)
            rclpy.spin_until_future_complete(_node, future)
            client.destroy()
            if future.result() is not None and future.result().values:
                value = future.result().values[0].value
                return UnifiedParameter(value)
    except Exception as e:
        print(f"Failed to get parameter '{param_name}': {e}")

    return UnifiedParameter(default)

@__is_node_initiialized
def delete_param(param_name):
    """
        Deletes a parameter from the node.

        Args:
            param_name (str): Name of the parameter.
        """
    _node.undeclare_parameter(param_name)

@__is_node_initiialized
def subscriber_count_per_topic(topic_name):
    """
    Counts the number of subscriptions for a topic.

    Args:
        topic_name (str): Name of the topic.

    Returns:
        int: Number of subscriptions.
    """

    return _node.count_subscribers(topic_name)

@__is_node_initiialized
def publisher_count_per_topic(topic_name):
    """
    Counts the number of publishers for a topic.

    Args:
        topic_name (str): Name of the topic.

    Returns:
        int: Number of publishers.
    """

    return _node.count_publishers(topic_name)

@__is_node_initiialized
def create_publisher(topic, msg_type):
    """
    Creates a publisher and returns it as a UnifiedPublisher.

    Args:
        topic (str): Name of the topic.
        msg_type (type): Type of the message.

    Returns:
        UnifiedPublisher: The created publisher.
    """
    print("ROS2: Creating Publisher")

    publisher = ROS2Publisher(_node, topic, msg_type)
    return publisher

@__is_node_initiialized
def create_subscriber(topic, msg_type, execute_cb):
    """
    Creates a subscriber and returns it as a UnifiedSubscriber.

    Args:
        topic (str): Name of the topic.
        msg_type (type): Type of the message.
        execute_cb (callable): Callback function for the subscription.

    Returns:
        UnifiedSubscription: The created subscription.
    """
    print("ROS2: Creating Subscriber")

    subscription = ROS2Subscriber(_node, topic, msg_type, execute_cb)
    return subscription

@__is_node_initiialized
def create_service(name,service_class, execute_cb):
    """
    Creates a service and returns it as a UnifiedService.

    Args:
        name (str): Name of the service.
        service_class (type): Class of the service.
        execute_cb (callable): Callback function for the service.

    Returns:
        UnifiedService: The created service.
    """

    service = ROS2Service(_node, name, service_class, execute_cb)
    return service

@__is_node_initiialized
def call_service(service_name, service_class, request):
    """
    Calls a service provided by another node.

    Args:
        service_name (str): Name of the service.
        service_class (type): Class of the service.
        request: Request object for the service.

    Returns:
        Response: Response from the service.
    """

    client = _node.create_client(service_class, service_name)

    client.wait_for_service()

    future = client.call_async(request)
    rclpy.spin_until_future_complete(_node, future)

    client.destroy()
    if future.result() is not None:
        return future.result()
    else:
        return None


@__is_node_initiialized
def get_all_nodes():
    """
    Gets all the reachable nodes in the network.

    Returns:
        list: List of node names.
    """

    node_names = _node.get_node_names()
    return node_names


@__is_node_initiialized
def get_all_services():
    """
    Gets all the reachable services in the network.

    Returns:
        list: List of service names and types.
    """

    return  [name for name , type in _node.get_service_names_and_types()]

@__is_node_initiialized
def get_all_topics():
    """
    Gets all the reachable topics in the network.

    Returns:
        list: List of topic names and types.
    """

    return  [name for name , type in _node.get_topic_names_and_types()] # Not

@__is_node_initiialized
def spin():
    """
       Spins the node.
    """

    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()

def spin_once():
    """
    Spins the node once.
    """

    rclpy.spin_once(_node)