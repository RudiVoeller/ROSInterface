import rospy
import rosnode
import rosgraph
import rosservice
import rostopic
import subprocess
import time

from docutils.nodes import topic

from .param.unified_param import UnifiedParameter
from ros_wrapper.action_client.ros1_action_client import ROS1ActionClient
from ros_wrapper.action_server.ros1_action_server import ROS1ActionServer
from .publisher.ros1_publisher import ROS1Publisher
from .service.ros1_service import ROS1Service
from .service.unified_service import UnifiedService
from .subscription.ros1_subscription import ROS1Subscription
from .subscription.unified_subscription import UnifiedSubscription
from ros_wrapper.action_client.unified_action_client import UnifiedActionClient
from ros_wrapper.action_server.unified_action_server import UnifiedActionServer
from ros_wrapper.publisher.unified_publisher import UnifiedPublisher

def init_node(name, anonymous=False):
    """
       Create node and put the object in the member variable node.

       \param node_name Name of the node.
       \param anonymous If True, the node name will be made unique by adding random characters.
    """
    start_roscore()
    rospy.init_node(name, anonymous=anonymous)

def create_action_server(action_name, action_type, execute_cb):
    """
           Create the action server and returns as UnifiedActionServer.

           \param action_name Name of the action.
           \param action_type Type of the action.
           \param execute_cb Callback function to execute the action.
           \return UnifiedActionServer object or None if node is not initialized.
    """
    server = ROS1ActionServer(action_name, action_type, execute_cb)
    rospy.loginfo(f"Action-Server '{action_name}' in ROS 1 gestartet.")
    return UnifiedActionServer(server)

def create_action_client(action_name, action_type):
    """
          Create the action client and returns as UnifiedActionClient.

          \param action_name Name of the action.
          \param action_type Type of the action.
          \return UnifiedActionClient object or None if node is not initialized.
    """
    client = ROS1ActionClient(action_name, action_type)
    return UnifiedActionClient(client)
#Available in ROS 2????

# FIXME: Only in ROS 1
def set_shutdown_hook(shutdown_hook):
    rospy.on_shutdown(shutdown_hook)

def set_param(param_name, value):
    """
           Sets a parameter on the parameter server.

           \param name Name of the parameter.
           \param value Value of the parameter.
    """
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    rospy.set_param(param_name, value)


def get_param(param_name, default=None):
    """
            Gets the parameter from the parameter server.

            \param param_name Name of the parameter.
            \param default Default value if the parameter is not found.
            \return UnifiedParameter object or None if node is not initialized.
    """
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    value = rospy.get_param(param_name, default)
    return UnifiedParameter(value)

def delete_param(param_name):
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    rospy.delete_param(param_name)

def subscription_count_per_topic(topic_name):
    """
            Counts the amount of subscriptions for one topic.

            \param topic_name Name of the topic.
            \return Number of subscriptions or None if node is not initialized.
    """
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None

    if not topic_name.startswith('/'):
        topic_name = '/' + topic_name
    strlist = rostopic.get_topic_type(topic_name)
   

    try:
        # Create a temporary publisher for the topic
        print(rostopic.get_topic_type(topic_name))
        temp_publisher = rospy.Publisher(topic_name, strlist[0], queue_size=10)
        rospy.sleep(1)  # Give some time for connections to be established

        # Get the number of connections (subscribers)
        num_connections = temp_publisher.get_num_connections()
        return num_connections
    except Exception as e:
        print(f"Failed to get subscription count for topic '{topic_name}': {e}")
        return None
    # Maybe get over get_num_connections of Publisher, but then need the msg_type

def publisher_count_per_topic(topic_name): # currently not working
    """
            Counts the amount of publishers for one topic.

            \param topic_name Name of the topic.
            \return Number of publishers or None if node is not initialized.
    """
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
# Maybe get over get_num_connections of Subscriber, but then need the msg_type

def create_publisher(topic, msg_type):
    """
            Creates a publisher and returns as UnifiedPublisher.

            \param topic Name of the topic.
            \param msg_type Type of the message.
            \return UnifiedPublisher object or None if node is not initialized.
    """
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    publisher = ROS1Publisher(topic, msg_type)
    return UnifiedPublisher(publisher)

def create_subscriber(topic, msg_type, callback):
    """
            Creates a subscriber and returns as UnifiedSubscriber.

            \param topic Name of the topic.
            \param msg_type Type of the message.
            \param callback Callback function for the subscription.
            \return UnifiedSubscription object or None if node is not initialized.
    """
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    subscription = ROS1Subscription(topic, msg_type, callback)
    return UnifiedSubscription(subscription)

def create_service(name,service_class, handler):
    """
            Creates a service - the handler object is used as a callback.

            \param name Name of the service.
            \param service_class Class of the service.
            \param handler Callback function for the service.
    """
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    service = ROS1Service(name, service_class, handler)
    return UnifiedService(service)

#TODO: Need to be tested
def call_service(service_name, service_type, *args):
    """
           Calls a service provided by another node.

           \param service_name Name of the service.
           \param service_type Type of the service.
           \param args Arguments for the service call.
           \return Response from the service or None if node is not initialized.
    """
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    rospy.wait_for_service(service_name)
    try:
        # Service-Proxy erstellen
        service_proxy = rospy.ServiceProxy(service_name, service_type)

        # Service aufrufen mit den Argumenten
        response = service_proxy(*args)
        return response
    except rospy.ServiceException as e:
        print(f"Service call dont work: {e}")

def get_all_nodes():
    """
          Gets all the reachable nodes in the network.

          \return List of node names or an empty list if node is not initialized.
    """
    return rosnode.get_node_names()

def get_all_services():
    """
           Gets all the reachable services in the network.

           \return List of service names and types or an empty list if node is not initialized.
    """
    return rosservice.get_service_list()

def get_all_topics():
    """
           Gets all the reachable topics in the network.

           \return List of topic names and types or an empty list if node is not initialized.
    """
    return rospy.get_published_topics()

def spin():
    """
           Spins the node.
    """
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    rospy.spin()

#############INTERN################

""" Starts the roscore if it is not started yet"""
def start_roscore():
    try:
        # Überprüfen, ob der Master läuft
        if rosgraph.is_master_online():
            print("ROS Master is already running.")
        else:
            print("No ROS Master detected. Starting roscore...")
            # Starte `roscore` in einem neuen Prozess
            subprocess.Popen(['roscore'])
            # Warte, bis der Master verfügbar ist
            time.sleep(5)  # Kurz warten, damit der Master vollständig gestartet ist
    except Exception as e:
        print(f"Failed to start roscore: {e}")

""" Checks if a node is initialized """
def is_node_initialized():
    try:
        rospy.get_node_uri()  # Prüft, ob der Node bereits initialisiert wurde
        return True
    except rospy.exceptions.ROSException:
        return False