class UnifiedPublisher:
    def __init__(self, publisher):
        self.publisher = publisher

    def publish(self, msg):
        self.publisher.publish(msg)