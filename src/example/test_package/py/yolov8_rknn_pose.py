# Ultralytics YOLO 🚀, AGPL-3.0 license

import argparse

import cv2
import numpy as np
from rknnlite.api import RKNNLite
import os
import shutil
import time
import json
import imageio.v3 as iio
import tqdm
import threading
from concurrent.futures import ThreadPoolExecutor


def np_sigmoid(x):
    return 1 / (1 + np.exp(-x))


def make_anchors(feats, strides, grid_cell_offset=0.5):
    """Generate anchors from features."""
    anchor_points, stride_tensor = [], []
    assert feats is not None

    for i, stride in enumerate(strides):
        _, _, h, w = feats[i].shape
        # 创建网格坐标
        sx = np.arange(w, dtype=np.float32) + grid_cell_offset  # shift x
        sy = np.arange(h, dtype=np.float32) + grid_cell_offset  # shift y
        # 使用 numpy 的 meshgrid 创建网格
        sy, sx = np.meshgrid(sy, sx, indexing="ij")
        # 堆叠并重塑坐标
        anchor_points.append(np.stack((sx, sy), -1).reshape(-1, 2))
        # 创建对应的 stride tensor
        stride_tensor.append(np.full((h * w, 1), stride, dtype=np.float32))

    # 连接所有 anchor points 和 stride tensors
    return np.concatenate(anchor_points), np.concatenate(stride_tensor)


def dist2bbox(distance, anchor_points, xywh=True, dim=-1):
    """Transform distance(ltrb) to box(xywh or xyxy)."""
    # 将distance在指定维度上分成两半，分别得到lt和rb
    split_size = distance.shape[dim] // 2
    lt = np.split(distance, [split_size], dim)[0]
    rb = np.split(distance, [split_size], dim)[1]

    x1y1 = anchor_points - lt
    x2y2 = anchor_points + rb

    if xywh:
        c_xy = (x1y1 + x2y2) / 2
        wh = x2y2 - x1y1
        return np.concatenate((c_xy, wh), axis=dim)  # xywh bbox
    return np.concatenate((x1y1, x2y2), axis=dim)  # xyxy bbox


class MeanAverageTime:
    def __init__(self):
        self.total_time = 0
        self.count = 0

    def update(self, time):
        self.total_time += time
        self.count += 1

    def get(self):
        return self.total_time / self.count


class YOLOv8Pose:
    """YOLOv8 object detection model class for handling inference and visualization."""

    def __init__(
        self,
        rknn_model,
        input_size,
        confidence_thres,
        iou_thres,
        stride=np.array([8, 16, 32]),
        reg_max=16,
        nc=1,
        no=65,
        proj=None,
    ):
        """
        Initializes an instance of the YOLOv8 class.

        Args:
            rknn_model: Path to the RKNN model.
            input_size: Input size of the model.
            confidence_thres: Confidence threshold for filtering detections.
            iou_thres: IoU (Intersection over Union) threshold for non-maximum suppression.
        """
        self.rknn_lite = RKNNLite()
        self.rknn_lite.load_rknn(rknn_model)
        self.rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_ALL)
        self.input_width = input_size[0]
        self.input_height = input_size[1]

        self.confidence_thres = confidence_thres
        self.iou_thres = iou_thres

        self.reg_max = reg_max  # dfl回归最大值
        self.nc = nc  # 类别数
        self.no = no  # 输出通道数
        self.proj = proj  # 投影向量
        self.stride = stride  # 步长

        self.kpt_shape = [17, 3]
        self.shape = [1, 65, 80, 80]

        self.classes = {"0": "person"}
        # Generate a color palette for the classes
        self.color_palette = np.random.uniform(0, 255, size=(len(self.classes), 3))
        self.preprocess_time = MeanAverageTime()
        self.postprocess_time = MeanAverageTime()
        self.inference_time = MeanAverageTime()
        self.skeleton = [
            [15, 13],
            [13, 11],
            [16, 14],
            [14, 12],
            [11, 12],
            [5, 11],
            [6, 12],
            [5, 6],
            [5, 7],
            [6, 8],
            [7, 9],
            [8, 10],
            [1, 2],
            [0, 1],
            [0, 2],
            [1, 3],
            [2, 4],
            [3, 5],
            [4, 6],
        ]

    def letterbox(self, img, new_shape=(640, 640), color=(114, 114, 114)):
        # Resize and pad image to new shape
        shape = img.shape[:2]  # current shape [height, width]

        # 计算缩放比例
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        ratio = r, r  # width, height ratios

        # 计算需要填充的宽度和高度
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

        # 均匀分配填充
        dw /= 2
        dh /= 2

        # 调整图像大小
        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)

        # 计算填充的上下左右位置
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

        # 添加填充
        img = cv2.copyMakeBorder(
            img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
        )
        self.dw = dw
        self.dh = dh
        self.ratio = ratio

        return img

    def preprocess(self, image: np.ndarray, image_path: str | None = None):
        """
        Preprocesses the input image before performing inference, with resize and padding (using letterbox)

        Parameters
        ---------------
        image: np.ndarray
            the input image, in RGB format
        image_path: str | None
            the path of the input image, which is used to compute image id

        Returns
        ---------------
        output_image: np.ndarray
            the preprocessed image data ready for inference, in NHWC format
        """
        # Read the input image using OpenCV
        if image_path is not None:
            self.image_id = int(os.path.basename(image_path).split(".")[0])
        self.img = image
        self.img_height, self.img_width = image.shape[:2]

        img = self.letterbox(image, (self.input_width, self.input_height))
        image_data = np.expand_dims(img, axis=0)

        # Return the preprocessed image data
        return image_data

    def postprocess(self, dets, kpts):
        """
        Performs post-processing on the model's output to extract bounding boxes, scores, and class IDs.

        Parameters
        ---------------
        dets: np.ndarray
            the output of the model
        kpts: np.ndarray
            the output of the model

        Returns
        ---------------
        output_boxes: list[tuple[float, int, tuple[float, float, float, float]]]
            the output boxes with score, class id and box
        output_kpts: list[np.ndarray]
            the output kpts
        image_id: int
            the image id
        """
        # Transpose and squeeze the output to match the expected shape
        dets = np.transpose(np.squeeze(dets))
        kpts = np.transpose(np.squeeze(kpts), (2, 0, 1))
        # Get the number of rows in the outputs array
        rows = dets.shape[0]
        # Lists to store the bounding boxes, scores, and class IDs of the detections
        boxes = []
        scores = []
        class_ids = []

        poses = []

        # Iterate over each row in the outputs array
        for i in range(rows):
            # Extract the class scores from the current row
            classes_scores = dets[i][4:]

            # Find the maximum score among the class scores
            max_score = np.amax(classes_scores)

            # If the maximum score is above the confidence threshold
            if max_score >= self.confidence_thres:
                # Get the class ID with the highest score
                class_id = np.argmax(classes_scores)

                # Extract the bounding box coordinates from the current row
                x, y, w, h = dets[i][0], dets[i][1], dets[i][2], dets[i][3]

                cur_kpts = kpts[i]

                # Calculate the scaled coordinates of the bounding box according self.dw and self.dh
                left = float((x - w / 2) - self.dw) / self.ratio[0]
                top = float((y - h / 2) - self.dh) / self.ratio[1]
                width = float(w) / self.ratio[0]
                height = float(h) / self.ratio[1]

                cur_kpts[:, 0] = (cur_kpts[:, 0] - self.dw) / self.ratio[0]
                cur_kpts[:, 1] = (cur_kpts[:, 1] - self.dh) / self.ratio[1]

                # Add the class ID, score, and box coordinates to the respective lists
                class_ids.append(class_id)
                scores.append(max_score)
                boxes.append([left, top, width, height])
                poses.append(cur_kpts)

        # Apply non-maximum suppression to filter out overlapping bounding boxes
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.confidence_thres, self.iou_thres)
        output_boxes = []
        output_kpts = []
        # Iterate over the selected indices after non-maximum suppression
        for i in indices:
            # Get the box, score, and class ID corresponding to the index
            box = boxes[i]
            score = scores[i]
            class_id = class_ids[i]
            output_boxes.append((score.item(), class_id.item(), box))
            output_kpts.append(poses[i])
        return output_boxes, output_kpts

    def det_decode(self, x):
        if self.proj is None:
            self.proj = np.arange(self.reg_max)
        self.anchors, self.strides = (
            x.transpose() for x in make_anchors(x, self.stride, 0.5)
        )
        shape = x[0].shape
        x_cat = np.concatenate([xi.reshape(shape[0], self.no, -1) for xi in x], axis=2)
        box = x_cat[:, : self.reg_max * 4, :]
        cls = x_cat[:, self.reg_max * 4 : self.reg_max * 4 + self.nc, :]
        box = np.transpose(box, (0, 2, 1))
        b, a, c = box.shape
        box = box.reshape(b, a, 4, c // 4)
        box = np.exp(box) / np.sum(np.exp(box), axis=3, keepdims=True)  # softmax

        box = np.matmul(box, self.proj)
        box = np.transpose(box, (0, 2, 1))
        dbox = (
            dist2bbox(box, np.expand_dims(self.anchors, 0), xywh=True, dim=1)
            * self.strides
        )

        y = np.concatenate((dbox, np_sigmoid(cls)), axis=1)

        return y

    def kpts_decode(self, bs, kpts):
        nk = self.kpt_shape[0] * self.kpt_shape[1]
        ndim = self.kpt_shape[1]
        kpt = np.concatenate([k.reshape(bs, nk, -1) for k in kpts], axis=-1)

        y = kpt.reshape(bs, *self.kpt_shape, -1)
        a = (y[:, :, :2] * 2.0 + (self.anchors - 0.5)) * self.strides

        if ndim == 3:
            # 连接坐标和置信度分数
            a = np.concatenate((a, np_sigmoid(y[:, :, 2:3])), axis=2)
        return a

    def detect(self, image: np.ndarray):
        """
        Performs inference using an RKNN model and returns the output image with drawn detections.

        Parameters
        ---------------
        image: np.ndarray
            the input image, in RGB format

        Returns
        ---------------
            output_img: The output image with drawn detections.
        """

        # Preprocess the image data
        preprocess_start = time.time()
        img_data = self.preprocess(image)
        preprocess_end = time.time()
        self.preprocess_time.update(preprocess_end - preprocess_start)

        inference_start = time.time()
        raw_outputs = self.rknn_lite.inference(inputs=[img_data], data_format=["nhwc"])
        inference_end = time.time()
        self.inference_time.update(inference_end - inference_start)

        postprocess_start = time.time()
        dets = self.det_decode(raw_outputs[:3])
        kpts = self.kpts_decode(1, raw_outputs[3:])
        det_results = self.postprocess(dets, kpts)  # output image
        postprocess_end = time.time()
        self.postprocess_time.update(postprocess_end - postprocess_start)

        return det_results

    def draw_kpts(self, img, kpts, score_thres=0.3):
        for kpt in kpts:
            if kpt[2] > score_thres:
                cv2.circle(img, (int(kpt[0]), int(kpt[1])), 3, (0, 0, 255), -1)
        for edge in self.skeleton:
            if kpts[edge[0]][2] > score_thres and kpts[edge[1]][2] > score_thres:
                cv2.line(
                    img,
                    (int(kpts[edge[0]][0]), int(kpts[edge[0]][1])),
                    (int(kpts[edge[1]][0]), int(kpts[edge[1]][1])),
                    (0, 255, 0),
                    2,
                )

    def draw_detections(self, img, box, score, class_id):
        x1, y1, w, h = box
        color = self.color_palette[class_id]
        cv2.rectangle(img, (int(x1), int(y1)), (int(x1 + w), int(y1 + h)), color, 2)
        label = f"{self.classes[str(class_id)]}: {score:.2f}"
        (label_width, label_height), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        label_x = x1
        label_y = y1 - 10 if y1 - 10 > label_height else y1 + 10
        cv2.rectangle(
            img,
            (int(label_x), int(label_y - label_height)),
            (int(label_x + label_width), int(label_y + label_height)),
            color,
            cv2.FILLED,
        )
        cv2.putText(
            img,
            label,
            (int(label_x), int(label_y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )


if __name__ == "__main__":
    # rknn_model = r"/data/code/psf_ros2_ws/tmp/models/rknn/yolov8s-pose-fp-bs1.rknn"
    rknn_model = r"/data/code/psf_ros2_ws/tmp/models/rknn/yolov8s-pose-ptq-bs1.rknn"
    # rknn_model = r"/data/code/psf_ros2_ws/tmp/models/rknn/yolov8n-pose-ptq-bs1.rknn"
    fn_image = r"/data/code/psf_ros2_ws/data/ori_img.jpg"
    output_dir = r"/data/code/psf_ros2_ws/tmp/output/rknn-pose"

    # read image
    img = iio.imread(fn_image)

    # load model
    conf_thres = 0.3
    iou_thres = 0.5
    import asyncio

    # Create 3 detectors
    detector1 = YOLOv8Pose(rknn_model, (640, 640), conf_thres, iou_thres)
    detector2 = YOLOv8Pose(rknn_model, (640, 640), conf_thres, iou_thres)
    detector3 = YOLOv8Pose(rknn_model, (640, 640), conf_thres, iou_thres)
    detector_list: list[YOLOv8Pose] = [detector1, detector2, detector3]
    num_iterations = 30

    async def detect_task(detector, img):
        return detector.detect(img)

    def run_detections():
        async def _run_detections_async():
            det_results = []
            kpts_results = []
            for i in tqdm.tqdm(range(num_iterations)):
                # Create coroutines for each detector
                tasks = []
                for detector in detector_list:
                    tasks.append(detect_task(detector, img))

                # Wait for all tasks to complete
                results = await asyncio.gather(*tasks)

                # Use results from first detector
                det_results, kpts_results = results[0]

            return det_results, kpts_results

        return asyncio.run(_run_detections_async())

    # Run the detection loop
    det_results, kpts_results = run_detections()

    canvas = img.copy()
    for (score, class_id, box), kpts in zip(det_results, kpts_results):
        detector1.draw_detections(canvas, box, score, class_id)
        detector1.draw_kpts(canvas, kpts)

    _, filename = os.path.split(fn_image)
    iio.imwrite(os.path.join(output_dir, f"{filename}-pose.jpg"), canvas)

    total_time_per_detector = [
        d.inference_time.get() * num_iterations for d in detector_list
    ]
    print(f"Total time per detector: {total_time_per_detector}")
    print(
        f"Total average time: {np.sum(total_time_per_detector) / (num_iterations * len(detector_list))}"
    )
    # print(f"Preprocess time: {detector1.preprocess_time.get()}")
    # print(f"Inference time: {detector1.inference_time.get()}")
    # print(f"Postprocess time: {detector1.postprocess_time.get()}")
    print(f"Model: {rknn_model}")
