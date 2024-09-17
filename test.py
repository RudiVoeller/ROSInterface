import ros_wrapper as ros
from std_msgs.msg import String


def callback(msg):
    print(f"Received: {msg.data}")


def main():
    ros.init_node('receiver', False, True)

    #publisher = ros.create_publisher('chatter', String)
    ros.create_subscriber('TEST', String, callback)
    ros.create_service('test2', String, callback)


if __name__ == '__main__':
    main()
    ros.spin()