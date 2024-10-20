class UnifiedGoalHandle:
    def __init__(self):
        self._succeeded = False
        self._aborted = False
        self._canceled = False

    def succeed(self):
        self._succeeded = True

    def abort(self):
        self._aborted = True

    def cancel(self):
        self._canceled = True

    @property
    def is_succeeded(self):
        return self._succeeded

    @property
    def is_aborted(self):
        return self._aborted

    @property
    def is_canceled(self):
        return self._canceled