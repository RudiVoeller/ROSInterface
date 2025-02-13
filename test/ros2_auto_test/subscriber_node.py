import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
sys.path.append('/home/niklas/PycharmProjects/ROSConverter')
import ros_wrapper

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.create_subscription(
            String,
            'test_topic',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(f'Nachricht empfangen: {msg.data}')

def main(args=None):
    ros_wrapper.init_node("test_subscriber")

    ros_wrapper.create_subscriber("test_topic", String, lambda msg: print(msg.data))
    try:
        ros_wrapper.spin()
    finally:
        print("Shutting down")
        #ros_wrapper.shutdown()

if __name__ == '__main__':
    main()
