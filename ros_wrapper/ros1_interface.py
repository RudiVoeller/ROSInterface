import rospy
import rosgraph
import subprocess
import time
from std_msgs.msg import String


def set_shutdown_hook(shutdown_hook):
    rospy.on_shutdown(shutdown_hook)

def init_node(name, anonymous=False, log_level=rospy.INFO, verbose=False):
    if verbose:
        print("ROS 1: Init Node")
    start_roscore()
    rospy.init_node(name, anonymous=anonymous, log_level=log_level)

def get_param(param_name, default=None):
    return rospy.get_param(param_name, default)

def set_param(param_name, value):
    rospy.set_param(param_name, value)


def create_publisher(topic, msg_type, queue_size=10, latch=False, tcp_nodelay=False):
    print("ROS 1: Create Publisher")
    return rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=latch, tcp_nodelay=tcp_nodelay)

def create_subscriber(topic, msg_type, callback):
    print("ROS 1: Create Subscriber")
    return rospy.Subscriber(topic, msg_type, callback)

def spin():
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
