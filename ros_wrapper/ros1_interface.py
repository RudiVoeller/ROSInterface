import rospy
import rosnode
import rosgraph
import rosservice
import subprocess
import time
import xmlrpc.client

from .param.unified_param import UnifiedParameter
from ros_wrapper.action_client.ros1_action_client import ROS1ActionClient
from ros_wrapper.action_server.ros1_action_server import ROS1ActionServer
from .subscription.ros1_subscription import ROS1Subscription
from .subscription.unified_subscription import UnifiedSubscription
from ros_wrapper.action_client.unified_action_client import UnifiedActionClient
from ros_wrapper.action_server.unified_action_server import UnifiedActionServer
from ros_wrapper.publisher.unified_publisher import UnifiedPublisher


def create_action_server(action_name, action_type, execute_cb):
    server = ROS1ActionServer(action_name, action_type, execute_cb)
    rospy.loginfo(f"Action-Server '{action_name}' in ROS 1 gestartet.")
    return UnifiedActionServer(server)

def create_action_client(action_name, action_type):
    client = ROS1ActionClient(action_name, action_type)
    return UnifiedActionClient(client)

#Available in ROS 2????
def set_shutdown_hook(shutdown_hook):
    rospy.on_shutdown(shutdown_hook)

def init_node(name, anonymous=False):
    start_roscore()
    rospy.init_node(name, anonymous=anonymous)

def get_param(param_name, default=None):
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    value = rospy.get_param(param_name, default)
    return UnifiedParameter(value)

def set_param(param_name, value):
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    rospy.set_param(param_name, value)

def delete_param(param_name):
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    rospy.delete_params(param_name)

def subscription_count_per_topic(topic_name, data_class):
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    master = xmlrpc.client.ServerProxy(rospy.get_master().getUri())
    code, message, topic_list = master.getSystemState()

    subscriber_count = 0
    for entry in topic_list[1]:  # Der zweite Eintrag enthält die Subscriber
        if entry[0] == topic_name:
            subscriber_count = len(entry[1])  # Subscriber zählen
            return subscriber_count
    return 0

def publisher_count_per_topic(topic_name):
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None

    # Master-Proxy erstellen
    master = xmlrpc.client.ServerProxy(rospy.get_master().getUri())

    # Topic, für das du die Publisher ermitteln willst
    topic_name = "/dein_topic"

    # Informationen zu dem Topic vom Master abrufen
    code, message, topic_list = master.getSystemState()

    # Topic-Liste durchgehen und nach dem entsprechenden Topic suchen
    publisher_count = 0
    for entry in topic_list[0]:  # Der erste Eintrag enthält die Publisher
        if entry[0] == topic_name:
            publisher_count = len(entry[1])
            return publisher_count
            break


def create_publisher(topic, msg_type, queue_size=10, latch=False, tcp_nodelay=False):
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    publisher = rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=latch, tcp_nodelay=tcp_nodelay)
    return UnifiedPublisher(publisher)

def create_subscriber(topic, msg_type, callback):
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    subscription = ROS1Subscription(topic, msg_type, callback)
    return UnifiedSubscription(subscription)

def create_service(name,service_class, handler):
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    return rospy.Service(name, service_class, handler)


def call_service(service_name, service_type, *args):
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
    return rosnode.get_node_names()

def get_all_services():
    return rosservice.get_service_list()


def get_all_topics():
    return rospy.get_published_topics()

def spin():
    if not is_node_initialized():
        print("ROS1: ERROR: First init a node")
        return None
    rospy.spin()

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

def is_node_initialized():
    try:
        rospy.get_node_uri()  # Prüft, ob der Node bereits initialisiert wurde
        return True
    except rospy.exceptions.ROSException:
        return False