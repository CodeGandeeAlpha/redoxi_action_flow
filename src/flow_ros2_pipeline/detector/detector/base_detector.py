from abc import ABC, abstractmethod

class BaseDetector(ABC):
    def __init__(self):
        self.model = None

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def infer(self, input_data) -> dict[str, list]:
        pass
