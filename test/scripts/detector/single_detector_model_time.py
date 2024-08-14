import cv2
from detector.ddq_detector import DdqDetrDetector
import time

ddq_model = DdqDetrDetector()
model_cfg = 'src/flow_ros2_pipeline/detector/configs/ddq/ddq-detr-4scale_swinl_8xb2-30e_coco.py'
weights = 'src/flow_ros2_pipeline/detector/models/ddq_detr_swinl_30e.pth'
ddq_model.init(model_cfg=model_cfg, model_path=weights, device=f'cuda:1', class_names=['person'])

cap = cv2.VideoCapture("/mnt/chengxiao/test.mp4")
frame_num = 0
start_time = time.time()
while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        break

    print(f"frame.shape: {frame.shape}")
    detections = ddq_model.infer(frame, 0.3)
    frame_num += 1

end_time = time.time()

# 计算总时间
total_time = end_time - start_time
print(f"Total inference time for the video: {total_time} seconds")
print(f"Average inference time per frame: {total_time / frame_num} seconds")

# 释放视频捕获对象
cap.release()
