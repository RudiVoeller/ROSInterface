import ros_wrapper as ros
from std_msgs.msg import String


def callback(msg):
    print(f"Received: {msg.data}")


def main():
    ros.init_node('my_node')

    publisher = ros.create_publisher('chatter', String)
    ros.create_subscriber('chatter', String, callback)


    while True:
        msg = String()
        msg.data = 'Hello World'
        publisher.publish(msg)


if __name__ == '__main__':
    main()
    ros.spin()