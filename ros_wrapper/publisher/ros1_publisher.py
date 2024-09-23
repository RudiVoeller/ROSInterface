import rospy

class ROS1Publisher:
    def __init__(self, topic, msg_type, queue_size=10):
        self.publisher = rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def unregister(self):
        self.publisher.unregister()

    def publish(self, msg):
        self.publisher.publish(msg)