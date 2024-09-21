import rospy

class ROS1Subscription:
    def __init__(self, topic, msg_type, callback):
        self.subscription = rospy.Subscriber(topic, msg_type, callback)

    def unregister(self):
        self.subscription.unregister()