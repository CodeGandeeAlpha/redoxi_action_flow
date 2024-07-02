import vineyard
from vineyard import ObjectID
import cv2
import numpy as np

# 连接到 Vineyard 客户端
client = vineyard.connect('/var/run/vineyard.sock')

obj_id = ObjectID('o00001f2ba055665c')
data_meta = client.get_meta(obj_id)
data_object = client.get_object(obj_id)

shape = eval(data_meta['shape_'])
mem = memoryview(data_object.member('buffer_'))[
        0 : int(np.prod(shape)) * np.dtype('uint8').itemsize
    ]
image_data = np.frombuffer(mem, dtype=np.uint8).reshape(shape)

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