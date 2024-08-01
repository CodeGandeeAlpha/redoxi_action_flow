import vineyard
from vineyard import ObjectID
import cv2
import numpy as np

def get_img_by_v6d_id(v6d_client: vineyard.Client, v6d_id: int) -> np.ndarray:
    obj_id = ObjectID(v6d_id)
    data_meta = v6d_client.get_meta(obj_id)
    data_object = v6d_client.get_object(obj_id)

    shape = eval(data_meta['shape_'])
    mem = memoryview(data_object.member('buffer_'))[
            0 : int(np.prod(shape)) * np.dtype('uint8').itemsize
        ]
    image_data = np.frombuffer(mem, dtype=np.uint8).reshape(shape)

    return image_data

# 连接到 Vineyard 客户端
client = vineyard.connect('/var/run/vineyard.sock')

cv2.imwrite('v6d_test.jpg', get_img_by_v6d_id(client, 4221158535414544))

assert False
shared_tensor = client.get('o0123139ffecfd8de')
image_data = np.frombuffer(shared_tensor.data, dtype=shared_tensor.dtype).reshape(
                    shared_tensor.shape)

# 检查数据形状和尺寸
print("Image data shape:", image_data.shape)

# 检查数据类型
print("Image data type:", image_data.dtype)

print("Image data:", image_data)

# # # 将数据转换为 OpenCV 的 Mat 格式
# image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

cv2.imwrite('test.jpg', image_data)

print(shared_tensor)
print(type(shared_tensor))