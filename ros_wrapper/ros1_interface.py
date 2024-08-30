import rospy
from std_msgs.msg import String

def init_node(name, anonymous=True):
    rospy.init_node(name, anonymous=anonymous)

def create_publisher(topic, msg_type, queue_size=10):
    return rospy.Publisher(topic, msg_type, queue_size=queue_size)

def create_subscriber(topic, msg_type, callback):
    return rospy.Subscriber(topic, msg_type, callback)

def spin():
    rospy.spin()