import vineyard
import numpy as np

def create_v6d_client(socket : str ="/var/run/vineyard.sock") -> vineyard.Client:
    v6d_client = vineyard.connect(socket)
    return v6d_client

def get_img_by_v6d_id(v6d_client: vineyard.Client, v6d_id: int) -> np.ndarray:
    obj_id = vineyard.ObjectID(v6d_id)
    data_meta = v6d_client.get_meta(obj_id)
    data_object = v6d_client.get_object(obj_id)

    shape = eval(data_meta['shape_'])
    mem = memoryview(data_object.member('buffer_'))[
            0 : int(np.prod(shape)) * np.dtype('uint8').itemsize
        ]
    image_data = np.frombuffer(mem, dtype=np.uint8).reshape(shape)

    return image_data
