from attr import define, field
import numpy as np

@define(kw_only=True)
class DetectionResult:
    class_id : int = field()
    class_name : str = field()
    xyxy : np.ndarray = field()
    score : float = field(default=0.0)

class BaseDetector():
    def init(self, *args, **kwargs):
        raise NotImplementedError()

    def infer(self, input_data, *args, **kwargs) -> list[list[DetectionResult]]:
        raise NotImplementedError()

    # @property
    # def model(self):
    #     raise NotImplementedError()
