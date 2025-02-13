import time

from std_msgs.msg import String

import sys
sys.path.append('/home/niklas/PycharmProjects/ROSConverter')
import ros_wrapper

def main(args=None):
    ros_wrapper.init_node("test_publisher")
    publisher = ros_wrapper.create_publisher("test_topic", String)
    time.sleep(3)
    msg = String()
    msg.data = "Hallo Welt!"
    publisher.publish(msg)

    try:
        ros_wrapper.spin()
    finally:
        print("Shutting down")
        #ros_wrapper.shutdown()

if __name__ == '__main__':
    main()
