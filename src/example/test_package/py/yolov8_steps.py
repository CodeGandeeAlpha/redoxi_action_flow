import onnxruntime as ort
from yolov8_example import YOLOv8pose
import cv2
import numpy as np
import torch
from vis_utils import imshow

fn_model = r"/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8n-pose-dynbatch.onnx"
fn_image = r"/soft/workspace/code/psf_ros2_ws/data/pose-sample.jpg"
confidence_thres = 0.5
iou_thres = 0.5
keypoint_vis_thres = 0.5

model = YOLOv8pose(fn_model, fn_image, confidence_thres, iou_thres, keypoint_vis_thres)

# Create an inference session using the ONNX model and specify execution providers
session = ort.InferenceSession(
    fn_model, providers=["CUDAExecutionProvider", "CPUExecutionProvider"]
)

# Get the model inputs
model_inputs = session.get_inputs()

# Store the shape of the input for later use
input_shape = model_inputs[0].shape
model.input_image = fn_image
model.input_width = input_shape[2]
model.input_height = input_shape[3]

# preprocess image
# read image
src_image = cv2.imread(fn_image)
model.img = src_image
# Get the height and width of the input image
model.img_height, model.img_width = model.img.shape[:2]

# Convert the image color space from BGR to RGB
img = cv2.cvtColor(model.img, cv2.COLOR_BGR2RGB)

# Resize the image to match the input shape
img = cv2.resize(img, (model.input_width, model.input_height))

# Normalize the image data by dividing it by 255.0
image_data = np.array(img) / 255.0

# Transpose the image to have the channel dimension as the first dimension
image_data = np.transpose(image_data, (2, 0, 1))  # Channel first

# Expand the dimensions of the image data to match the expected input shape
image_data = np.expand_dims(image_data, axis=0).astype(np.float32)

img_hwc = torch.tensor(image_data).squeeze(0).permute(1, 2, 0).numpy()
imshow(img_hwc)

# # Run inference using the preprocessed image data
outputs = session.run(None, {model_inputs[0].name: image_data})

vis_image = model.postprocess(src_image, outputs)

imshow(vis_image)
