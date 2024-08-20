import logging
import cv2
import uuid
import time
from ros2_easy_test import ROS2TestEnvironment, with_launch_file

import unittest
from unittest import TestCase
from abc import ABC

from psg_actions.action import ProcessFrame
from psg_common.utilities import create_v6d_client


logging.basicConfig(level=logging.DEBUG)

v6d_client = create_v6d_client()


class SharedTestCases(ABC):
    def test_action_by_video(self, env: ROS2TestEnvironment) -> None:
        cap = cv2.VideoCapture("/mnt/chengxiao/test.mp4")
        frame_num = 0
        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                break

            print(f"frame.shape: {frame.shape}")
            v6d_id = v6d_client.put(frame)
            int_id = int(v6d_id)

            goal_msg = ProcessFrame.Goal()

            goal_msg.frame.cache.id_int = int_id
            goal_msg.frame.cache.has_int_id = True
            goal_msg.frame.frame_num = frame_num
            t_uuid = uuid.uuid4()
            goal_msg.detections_uuid.uuid = list(t_uuid.bytes)

            feedbacks, result = env.send_action_goal_and_wait_for_result(name='model_process_frame_action',
                                                                         goal=goal_msg,
                                                                         timeout_availability=100000,
                                                                         timeout_accept_goal=100000,
                                                                         timeout_get_result=100000)
            # logging.info(f"test_action_by_video() got feedbacks {feedbacks}")
            # logging.info(f"test_action_by_video() got result {result}")
            frame_num += 1

        cap.release()

        time.sleep(30)

class TestDetectorNode(SharedTestCases, TestCase):
    @with_launch_file("src/flow_ros2_pipeline/launch/single_detector_launch.py")
    def test_action_by_video(self, env: ROS2TestEnvironment) -> None:
        super().test_action_by_video(env)


if __name__ == "__main__":
    unittest.main()