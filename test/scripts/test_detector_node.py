from psg_common.utilities import create_v6d_client
import cv2
import os
import uuid
from ros2_msg_transform import dict_to_ros2_msg_from_ns

v6d_client = create_v6d_client()

img = cv2.imread("test.jpg")
v6d_id = v6d_client.put(img)
int_id = int(v6d_id)

action_name = "model_process_frame_action"
action_type = "ProcessFrame"


for i in range(1):

    sh = f"sh /mnt/chengxiao/code/psf_ros2_ws/test/send_frame_action.sh {int_id} {action_name} {action_type} {i} {list(uuid.uuid4().bytes)}"
    print(sh)
    os.system(sh)