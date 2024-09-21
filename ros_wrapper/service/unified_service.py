class UnifiedService:
    def __init__(self, service):
        self.service = service

    def shutdown(self):
        self.service.shutdown()