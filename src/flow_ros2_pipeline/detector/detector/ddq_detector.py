from flow_ros2_pipeline.detector.detector.base_detector import BaseDetector
from mmengine.config import Config
from mmdet.apis import DetInferencer
import numpy as np

class DdqDetrDetector(BaseDetector):
    def __init__(self, model_body, model_head):
        super().__init__()
        self.model_body = model_body
        self.model_head = model_head

    def init(self, model_cfg, model_path, device='cuda:0', class_names=None, palette=None, show_progress=False):
        '''
        Args:
            model_type: str
                Name of the detection framework (example: "yolov5", "mmdet", "detectron2")
            model_path: str
                Path of the detection model (ex. 'model.pt')
            device: str
                Device, "cpu" or "cuda:0" or any other cuda device
            image_size: int
                Inference input size.
            confidence_threshold: float
                All predictions with score < confidence_threshold will be discarded
        '''
        # Init model
        if isinstance(device, int):
            device = f'cuda:{device}'
        if device.startswith('cuda') or device.startswith('cpu'):
            self.device = device
        else:
            raise ValueError(f"Invalid device: {device}. Must be 'cuda:idx' or 'cpu'")
        cfg = Config.fromfile(model_cfg)
        self.model = DetInferencer(model=cfg, weights=model_path, device=device,
                                        palette=palette, show_progress=show_progress, scope=cfg['default_scope'])
        if self.device.startswith('cuda'):
            # Warm up
            self.inferencer(inputs=np.zeros((640, 640, 3), dtype=np.uint8))
        if class_names is None:
            self.class_names = ['body', 'head', 'face']

    def infer(self, input_data: np.ndarray):
        results_dict = self.model(inputs=input_data)
        return results_dict