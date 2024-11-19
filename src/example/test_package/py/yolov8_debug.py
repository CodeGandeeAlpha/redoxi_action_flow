import numpy as np
from vis_utils import imshow
import imageio.v3 as iio
import torch

fn_input_data = r"/soft/workspace/code/psf_ros2_ws/tmp/output/pose-input-0.npy"
input_data = np.load(fn_input_data)

img_hwc = torch.tensor(input_data).squeeze(0).permute(1, 2, 0).numpy()
imshow(img_hwc)
