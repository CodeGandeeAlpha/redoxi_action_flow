from attr import define, field
import numpy as np

@define(kw_only=True)
class PoseDetectionResult:
    keypoints : np.ndarray = field(factory=np.ndarray)
    scores : np.ndarray = field(factory=np.ndarray)

class BasePoseDetector():
    def init(self, *args, **kwargs):
        raise NotImplementedError()

    def infer(self, input_data, bboxes, *args, **kwargs) -> PoseDetectionResult:
        raise NotImplementedError()
