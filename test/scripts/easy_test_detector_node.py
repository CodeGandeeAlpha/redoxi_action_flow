from std_msgs.msg import String
import time
import logging
import cv2
import uuid
from psg_actions.action import ProcessFrame
from psg_common.utilities import create_v6d_client
from ros2_easy_test import ROS2TestEnvironment, with_launch_file, with_single_node

logging.basicConfig(level=logging.DEBUG)

v6d_client = create_v6d_client()

img = cv2.imread("test.jpg")
v6d_id = v6d_client.put(img)
int_id = int(v6d_id)

@with_launch_file("src/flow_ros2_pipeline/launch/launch.py",
                  watch_topics={"/easy_test_pub_topic": String})
def test_simple_publisher(env: ROS2TestEnvironment) -> None:
    for i in range(1000):
        logging.info(f"test_simple_publisher()[{i}]")
        response: String = env.assert_message_published("/easy_test_pub_topic", timeout=50)
        logging.info(f"test_simple_publisher()[{i}] got message {response.data}")
        # time.sleep(0.01)

@with_launch_file("src/flow_ros2_pipeline/launch/launch.py")
def test_calling_action(env: ROS2TestEnvironment) -> None:
    for i in range(1000):
        logging.info(f"test_calling_action()[{i}]")
        goal_msg = ProcessFrame.Goal()
        # psg_public_msgs/Frame frame
        # unique_identifier_msgs/UUID detections_uuid
        goal_msg.frame.cache.id_int = int_id
        goal_msg.frame.cache.has_int_id = True
        goal_msg.frame.frame_num = i
        t_uuid = uuid.uuid4()
        goal_msg.detections_uuid.uuid = list(t_uuid.bytes)
        logging.info(f"test_calling_action()[{i}] uuid {t_uuid}")

        feedbacks, result = env.send_action_goal_and_wait_for_result(name='model_process_frame_action',
                                                                     goal=goal_msg,
                                                                     timeout_availability=100000,
                                                                     timeout_accept_goal=100000,
                                                                     timeout_get_result=100000)
        logging.info(f"test_calling_action()[{i}] got feedbacks {feedbacks}")
        logging.info(f"test_calling_action()[{i}] got result {result}")
        time.sleep(0.01)


if __name__ == "__main__":
    test_calling_action()