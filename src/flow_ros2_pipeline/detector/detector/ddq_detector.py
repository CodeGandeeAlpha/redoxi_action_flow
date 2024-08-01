from detector.base_detector import BaseDetector, DetectionResult
from mmengine.config import Config
from mmdet.apis import DetInferencer
import numpy as np

class DdqDetrDetector(BaseDetector):
    def __init__(self):
        super().__init__()
        self.model = None
        self.model_head = None

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
            self.model(inputs=np.zeros((640, 640, 3), dtype=np.uint8))
        if class_names is None:
            self.class_names = ['body', 'head', 'face']
        else:
            self.class_names = class_names

    def infer(self, input_data: np.ndarray, pred_threshold=0.3):
        results_dict = self.model(inputs=input_data)
        results_list = []
        for predictions in results_dict['predictions']:
            preds = []
            for label, score, bbox in zip(predictions['labels'], predictions['scores'], predictions['bboxes']):
                if score < pred_threshold or label != 0:
                    continue
                det_result = DetectionResult(class_id=int(label), class_name=self.class_names[int(label)], xyxy=np.array(bbox), score=score)
                preds.append(det_result)
            results_list.append(preds)
        return results_list


if __name__ == "__main__":
    detector = DdqDetrDetector()
    model_cfg = '../configs/ddq/ddq-detr-4scale_swinl_8xb2-30e_coco.py'
    weights = '../models/ddq_detr_swinl_30e.pth'
    detector.init(model_cfg=model_cfg, model_path=weights, device='cuda:0', class_names=['person'])
    results = detector.infer(np.zeros((640, 640, 3), dtype=np.uint8), pred_threshold=0.01)
    print(results)