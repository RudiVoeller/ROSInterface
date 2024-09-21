import ros_wrapper as ros
from std_msgs.msg import String
import unittest
import time



if __name__ == '__main__':
    ros.init_node("test")
    publisher = ros.create_publisher('test_topic', String)
    print(ros.subscription_count_per_topic('test_topic'))

    try:
        while True:
            publisher.publish(String(data="Hello World"))
            time.sleep(1)  # Warte 1 Sekunde
            print(ros.subscription_count_per_topic('test_topic'))
    except KeyboardInterrupt:
        pass
    ros.spin()

    #unittest.main()