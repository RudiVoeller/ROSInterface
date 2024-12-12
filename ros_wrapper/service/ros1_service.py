import rospy

class ROS1Service:
    def __init__(self, name, service_class, handler):
        """
                Initializes the ROS1Service with the given service name, service class, and handler.

                Args:
                    name (str): The name of the service.
                    service_class (type): The type of the service.
                    handler (callable): The handler function for the service.
                """
        self.service = rospy.Service(name, service_class, handler)

    def shutdown(self):
        """
                Shuts down the service.
                """
        self.service.shutdown()