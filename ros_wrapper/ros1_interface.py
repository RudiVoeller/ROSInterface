import random
import string

import rospy
import rosnode
import rosgraph
import rosservice
import rostopic
import roslib
import inspect

from .param.unified_param import UnifiedParameter
from ros_wrapper.action_client.ros1_action_client import ROS1ActionClient
from ros_wrapper.action_server.ros1_action_server import ROS1ActionServer
from .publisher.ros1_publisher import ROS1Publisher
from .service.ros1_service import ROS1Service
from .subscription.ros1_subscriber import ROS1Subscriber


def __is_node_initiialized(a_func):

    def wrapTheFunction(*args, **kwargs):
        try:
            if not rosgraph.is_master_online():
                print("ROS1: ERROR: First start roscore")
                return None


            rospy.get_node_uri()  # Prüft, ob der Node bereits initialisiert wurde
        except rospy.exceptions.ROSException:
            print("ROS1: WARNING: No node initialized, Init node")
            letters = string.ascii_letters + string.digits
            name =  ''.join(random.choice(letters) for i in range(10))
            rospy.init_node(name, anonymous=True)

        return  a_func(*args, **kwargs)

    return wrapTheFunction

def init_node(name, anonymous=False):
    """
        Creates a ROS node.

        Args:
            name (str): Name of the node.
            anonymous (bool, optional): If True, the node name will be made unique by adding random characters. Defaults to False.
        """
    rospy.init_node(name, anonymous=anonymous)

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
    server = ROS1ActionServer(action_name, action_type, execute_cb)
    rospy.loginfo(f"Action-Server '{action_name}' in ROS 1 gestartet.")
    return server

def create_action_client(action_name, action_type):
    """
    Creates an action client and returns it as a UnifiedActionClient.

    Args:
        action_name (str): Name of the action.
        action_type (type): Type of the action.

    Returns:
        UnifiedActionClient: The created action client.
    """
    client = ROS1ActionClient(action_name, action_type)
    return client

def set_param(param_name, value):
    """
        Sets a parameter on the parameter server.

        Args:
            param_name (str): Name of the parameter.
            value: Value of the parameter.
        """

    rospy.set_param(param_name, value)

@__is_node_initiialized
def get_param(param_name, default=None):
    """
    Gets a parameter from the parameter server.

    Args:
        param_name (str): Name of the parameter.
        default: Default value if the parameter is not found. Defaults to None.

    Returns:
        UnifiedParameter: The parameter value.
    """

    value = rospy.get_param(param_name, default)
    return UnifiedParameter(value)

@__is_node_initiialized
def delete_param(param_name):
    """
        Deletes a parameter from the parameter server.

        Args:
            param_name (str): Name of the parameter.
        """
    rospy.delete_param(param_name)

@__is_node_initiialized
def subscriber_count_per_topic(topic_name):
    """
        Counts the number of subscriptions for a topic.

        Args:
            topic_name (str): Name of the topic.

        Returns:
            int: Number of subscriptions.
        """

    if not topic_name.startswith('/'):
        topic_name = '/' + topic_name
    try:
        strlist = rostopic.get_topic_type(topic_name)

        if strlist is None or strlist[0] is None:
            print(f"Failed to get subscription count for topic '{topic_name}': Topic not found")
            return 0
        package_name, message_name = strlist[0].split('/')

        # Import the message module dynamically
        roslib.load_manifest(package_name)
        msg_module = __import__(f"{package_name}.msg", fromlist=[message_name])

        # Get the message class
        msg_class = getattr(msg_module, message_name)


        # Create a temporary publisher for the topic
        #print(rostopic.get_topic_type(topic_name))
        temp_publisher = rospy.Publisher(topic_name, msg_class, queue_size=10)
        rospy.sleep(1)  # Give some time for connections to be established

        # Get the number of connections (subscribers)
        num_connections = temp_publisher.get_num_connections()
        # Unregister the temporary publisher
        temp_publisher.unregister()
        return num_connections
    except Exception as e:
        print(f"Failed to get subscriber count for topic '{topic_name}'")
        return None

@__is_node_initiialized
def publisher_count_per_topic(topic_name): # currently not working
    
    """
    Counts the number of publishers for a topic.

    Args:
        topic_name (str): Name of the topic.

    Returns:
        int: Number of publishers.
    """

    if not topic_name.startswith('/'):
        topic_name = '/' + topic_name
    try:
        strlist = rostopic.get_topic_type(topic_name)

        if strlist is None or strlist[0] is None:
            print(f"Failed to get publisher count for topic '{topic_name}': Topic not found")
            return 0
        package_name, message_name = strlist[0].split('/')

        # Import the message module dynamically
        roslib.load_manifest(package_name)
        msg_module = __import__(f"{package_name}.msg", fromlist=[message_name])

        # Get the message class
        msg_class = getattr(msg_module, message_name)

        # Create a temporary subscriber for the topic
        temp_subscriber = rospy.Subscriber(topic_name, msg_class, callback=lambda msg: None)
        rospy.sleep(1)  # Give some time for connections to be established

        # Get the number of connections (publishers)
        num_connections = temp_subscriber.impl.get_num_connections()
        # Unregister the temporary subscriber
        temp_subscriber.unregister()
        return num_connections
    except Exception as e:
        print(f"Failed to get publisher count for topic '{topic_name}': {e}")
        return None


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

    publisher = ROS1Publisher(topic, msg_type)
    return publisher

@__is_node_initiialized
def create_subscriber(topic, msg_type, callback):

    """
    Creates a subscriber and returns it as a UnifiedSubscriber.

    Args:
        topic (str): Name of the topic.
        msg_type (type): Type of the message.
        callback (callable): Callback function for the subscription.

    Returns:
        UnifiedSubscription: The created subscription.
    """

    subscription = ROS1Subscriber(topic, msg_type, callback)
    return subscription

@__is_node_initiialized
def create_service(name,service_class, handler):
    """
    Creates a service and returns it as a UnifiedService.

    Args:
        name (str): Name of the service.
        service_class (type): Class of the service.
        handler (callable): Callback function for the service.

    Returns:
        UnifiedService: The created service.
    """

    service = ROS1Service(name, service_class, handler)
    return service

@__is_node_initiialized
def call_service(service_name, service_type, *args):
    """
    Calls a service provided by another node.

    Args:
        service_name (str): Name of the service.
        service_type (type): Type of the service.
        *args: Arguments for the service call.

    Returns:
        Response: Response from the service.
    """

    rospy.wait_for_service(service_name)
    try:
        # Service-Proxy erstellen
        service_proxy = rospy.ServiceProxy(service_name, service_type)

        # Service aufrufen mit den Argumenten
        response = service_proxy(*args)

        response_attrs = inspect.getmembers(response, lambda a: not (inspect.isroutine(a)))
        response_values = [a for a in response_attrs if not (a[0].startswith('_'))]
        if response_values:
            return response_values[0][1]
        else:
            return None

    except rospy.ServiceException as e:
        print(f"Service call dont work: {e}")

def get_all_nodes():
    """
    Gets all the reachable nodes in the network.

    Returns:
        list: List of node names.
    """
    return rosnode.get_node_names()

def get_all_services():
    """
    Gets all the reachable services in the network.

    Returns:
        list: List of service names and types.
    """
    return rosservice.get_service_list()

def get_all_topics():
    """
    Gets all the reachable topics in the network.

    Returns:
        list: List of topic names and types.
    """
    return rospy.get_published_topics()

@__is_node_initiialized
def spin():
    """
           Spins the node.
    """
    rospy.spin()

def spin_once():
    """
    Spins the node once.
    """
    rospy.rostime.wallsleep(0.1)
    
def destroy_node():
    """
    Destroys the node.
    """
    rospy.signal_shutdown("Node destroyed")