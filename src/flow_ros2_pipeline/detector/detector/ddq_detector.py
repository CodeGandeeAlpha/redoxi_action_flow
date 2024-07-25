from base import BaseDetector

class DdqDetrDetector(BaseDetector):
    def __init__(self, model_body, model_head):
        super().__init__()
        self.model_body = model_body
        self.model_head = model_head

    def init(self):
        pass

    def infer(slef, input_data):
        pass