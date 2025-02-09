#!/usr/bin/env python

## Insert import path to the ROSConverter package
import sys
sys.path.append('/add/your/path/to/ROSInterface')
import ros_wrapper
from std_msgs.msg import String
import time



if __name__ == "__main__":
    ros_wrapper.init_node("publisher_node")
    ros_wrapper.create_publisher("test_topic", String)
    ros_wrapper.init_node("publisher_node")
    publisher = ros_wrapper.create_publisher("test_topic", String)

    msg = String()
    msg.data = "Test message from publisher"

    while True:
        publisher.publish(msg)
        print(ros_wrapper.get_all_topics())
        time.sleep(1)  # Publish every second

