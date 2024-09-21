class UnifiedActionServer:
    def __init__(self, server):
        self.server = server

    def publish_feedback(self, feedback):
        self.server.publish_feedback(feedback)

    def set_succeeded(self, result):
        self.server.set_succeeded(result)

    def set_aborted(self, result=None):
        self.server.set_aborted(result)

    def is_preempt_requested(self):
        return self.server.is_preempt_requested()

    def set_preempted(self):
        self.server.set_preempted()