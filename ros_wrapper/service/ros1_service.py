import rospy

class ROS1Service:
    def __init__(self, name, service_class, handler):
        """Initializes the ROS1Service.

                Args:
                    name (str): The name of the service.
                    service_class (Service): The class of the ROS service.
                    handler (function): The callback function for the service.
                """
        self.service = rospy.Service(name, service_class, handler)

    def shutdown(self):
        """Shuts down the service."""

        self.service.shutdown()