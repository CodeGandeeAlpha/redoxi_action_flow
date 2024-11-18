import PIL.Image
import numpy as np
from IPython.display import display


def imshow(img: np.ndarray | PIL.Image.Image):
    if isinstance(img, PIL.Image.Image):
        display(img)
    elif isinstance(img, np.ndarray):
        if img.dtype == np.float32 or img.dtype == np.float64:
            imgshow = (img * 255).astype(np.uint8)
        else:
            imgshow = img
        display(PIL.Image.fromarray(imgshow))
