import ros_wrapper as ros
from std_msgs.msg import String
import time

def init_publisher():
    ros.init_node("test_publisher_node")
    publisher = ros.create_publisher("test_topic", String)
    time.sleep(1)
    publisher.publish(String(data="Hello World"))
    ros.spin()
    publisher.unregister()

def init_subscriber(received_messages):
    ros.init_node("test_subscriber_node")
    def callback(msg):
        received_messages.append(msg.data)
    subscriber = ros.create_subscriber("test_topic", String, callback)
    ros.spin()
    subscriber.unregister()


