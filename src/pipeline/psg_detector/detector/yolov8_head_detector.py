from ultralytics import YOLO
from detector.base_detector import BaseDetector, DetectionResult


class YOLOv8HeadDetector(BaseDetector):
    def init(self, weights_path: str, task: str = 'detect', device: str = 'cuda:0'):
        self.model = YOLO(weights_path, task)
        self.device = device

    def infer(self, input_data, imgsz=800, max_det=1000, pred_threshold=0.60,
               show_labels=False, show_conf=True, save=False, augment=True):
        results = self.model(source=input_data, imgsz=imgsz, max_det=max_det, conf=pred_threshold,
                              show_labels=show_labels, show_conf=show_conf, save=save, device=self.device,
                                augment=augment)
        results_list = []
        for predictions in results:
            preds = []
            for bbox in predictions.boxes:
                score = bbox.conf.item()
                label = bbox.cls.item()
                box = bbox.xyxy.squeeze().cpu().numpy().astype(float)
                det_result = DetectionResult(class_id=int(label), class_name='head', xyxy=box, score=score)
                preds.append(det_result)
            results_list.append(preds)
        return results_list