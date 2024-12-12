class UnifiedService:
    def __init__(self, service):
        """
                Initializes the UnifiedService with the given ROS 1 or ROS 2 service.

                Args:
                    service (ROS1Service or ROS2Service): The underlying service instance.
                """
        self.service = service

    def shutdown(self):
        """
               Shuts down the service.
               """
        self.service.shutdown()