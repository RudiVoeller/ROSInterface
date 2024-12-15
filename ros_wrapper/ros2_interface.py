from math import e

import rclpy
from rclpy.node import Node

from ros_wrapper.param.unified_param import UnifiedParameter
from ros_wrapper.action_client.ros2_action_client import ROS2ActionClient
from ros_wrapper.action_server.ros2_action_server import ROS2ActionServer
from ros_wrapper.publisher.ros2_publisher import ROS2Publisher
from ros_wrapper.service.ros2_service import ROS2Service
from ros_wrapper.subscription.ros2_subscription import ROS2Subscription
from ros_wrapper.action_client.unified_action_client import UnifiedActionClient
from ros_wrapper.action_server.unified_action_server import UnifiedActionServer

_node = None

def __is_node_initiialized(a_func):

    def wrapTheFunction():

        if not _node:
            print("ROS2: ERROR: First init a node")
            return None

        a_func()


    return wrapTheFunction


def init_node(node_name, anonymous=False):
    """
    Create node and put the object in the member variable node.

    \param node_name Name of the node.
    \param anonymous If True, the node name will be made unique by adding random characters.
    """
    rclpy.init()
    global _node
    _node = Node(node_name)

@__is_node_initiialized
def create_action_server(action_name, action_type, execute_cb):
    """
        Create the action server and returns as UnifiedActionServer.

        \param action_name Name of the action.
        \param action_type Type of the action.
        \param execute_cb Callback function to execute the action.
        \return UnifiedActionServer object or None if node is not initialized.
    """


    server = ROS2ActionServer(_node, action_name, action_type, execute_cb)
    return server

@__is_node_initiialized
def create_action_client(action_name, action_type):
    """
      Create the action client and returns as UnifiedActionClient.

      \param action_name Name of the action.
      \param action_type Type of the action.
      \return UnifiedActionClient object or None if node is not initialized.
    """

    client = ROS2ActionClient(_node, action_name, action_type)
    return client

@__is_node_initiialized
def set_param(name, value):
    """
       Sets a parameter on the parameter server.

       \param name Name of the parameter.
       \param value Value of the parameter.
    """
    _node.declare_parameter(name, value)

@__is_node_initiialized
def get_param(param_name, default = None):
    """
        Gets the parameter from the parameter server.

        \param param_name Name of the parameter.
        \param default Default value if the parameter is not found.
        \return UnifiedParameter object or None if node is not initialized.
    """

    value = _node.get_parameter_or(param_name, default)
    return UnifiedParameter(value.value)

@__is_node_initiialized
def delete_param(param_name):

    _node.undeclare_parameter(param_name)

@__is_node_initiialized
def subscription_count_per_topic(topic_name):
    """
        Counts the amount of subscriptions for one topic.

        \param topic_name Name of the topic.
        \return Number of subscriptions or None if node is not initialized.
    """

    return _node.count_subscribers(topic_name)

@__is_node_initiialized
def publisher_count_per_topic(topic_name):
    """
        Counts the amount of publishers for one topic.

        \param topic_name Name of the topic.
        \return Number of publishers or None if node is not initialized.
    """

    return _node.count_publishers(topic_name)

@__is_node_initiialized
def create_publisher(topic, msg_type):
    """
        Creates a publisher and returns as UnifiedPublisher.

        \param topic Name of the topic.
        \param msg_type Type of the message.
        \return UnifiedPublisher object or None if node is not initialized.
    """
    print("ROS2: Creating Publisher")

    publisher = ROS2Publisher(_node, topic, msg_type)
    return publisher

@__is_node_initiialized
def create_subscriber(topic, msg_type, execute_cb):
    """
        Creates a subscriber and returns as UnifiedSubscriber.

        \param topic Name of the topic.
        \param msg_type Type of the message.
        \param callback Callback function for the subscription.
        \return UnifiedSubscription object or None if node is not initialized.
    """
    print("ROS2: Creating Subscriber")

    subscription = ROS2Subscription(_node, topic, msg_type, execute_cb)
    return subscription

@__is_node_initiialized
def create_service(name,service_class, execute_cb):
    """
        Creates a service - the handler object is used as a callback.

        \param name Name of the service.
        \param service_class Class of the service.
        \param execute_cb Callback function for the service.
    """

    service = ROS2Service(_node, name, service_class, execute_cb)
    return service

@__is_node_initiialized
def call_service(service_name, service_class, *args):
    """
       Calls a service provided by another node.

       \param service_name Name of the service.
       \param service_class Class of the service.
       \param args Arguments for the service call.
       \return Response from the service or None if node is not initialized.
    """

    client = _node.create_client(service_class, service_name)

    while not client.wait_for_service(timeout_sec=1.0):
        print(f"Service '{service_name}' not available, waiting...")

    request = service_class.Request()
    request_fields = [field for field in dir(request) if
                      not field.startswith('_') and not field.startswith("SLOT_TYPES") and not callable(getattr(request, field))]

    for field, value in zip(request_fields, args):
        setattr(request, field, value)

    future = client.call_async(request)
    rclpy.spin_until_future_complete(_node, future)

    if future.result() is not None:
        return future.result()
    else:
        return None


@__is_node_initiialized
def get_all_nodes():
    """
       Gets all the reachable nodes in the network.

       \return List of node names or an empty list if node is not initialized.
    """

    node_names_and_namespaces = _node.get_node_names_and_namespaces()
    return [name for name, namespace in node_names_and_namespaces]


@__is_node_initiialized
def get_all_services():
    """
        Gets all the reachable services in the network.

        \return List of service names and types or an empty list if node is not initialized.
    """

    return  [name for name , type in _node.get_service_names_and_types()]

@__is_node_initiialized
def get_all_topics():
    """
       Gets all the reachable topics in the network.

       \return List of topic names and types or an empty list if node is not initialized.
    """

    return  [name for name , type in _node.get_topic_names_and_types()]

@__is_node_initiialized
def spin():
    """
       Spins the node.
    """

    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()

