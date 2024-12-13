from abc import ABC, abstractmethod


class UnifiedService(ABC):

    @abstractmethod
    def shutdown(self):
        """
               Shuts down the service.
               """
        pass