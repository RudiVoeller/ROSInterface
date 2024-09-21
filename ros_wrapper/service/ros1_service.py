import rospy

class ROS1Service:
    def __init__(self, name, service_class, handler):
        self.service = rospy.Service(name, service_class, handler)

    def shutdown(self):
        self.service.shutdown()