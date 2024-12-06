class UnifiedService:
    def __init__(self, service):
        """Initializes the UnifiedService.

                Args:
                    service (Service): The ROS service (either ROS1 or ROS2).
                """
        self.service = service

    def shutdown(self):
        """Shuts down the service."""

        self.service.shutdown()