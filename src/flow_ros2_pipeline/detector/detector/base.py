class BaseDetector:
    def __init__(self):
        self.model = None

    def init(self):
        raise NotImplementedError("Subclasses must implement init method")

    def infer(self, input_data):
        raise NotImplementedError("Subclasses must implement infer method")

    # def detect(self, input_data):
    #     if self.model is None:
    #         self._model_init()
    #     return self._model_infer(input_data)