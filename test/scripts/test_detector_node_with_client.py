import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from psg_actions.action import ProcessFrame

v6d_client = create_v6d_client()

img = cv2.imread("test.jpg")
v6d_id = v6d_client.put(img)
int_id = int(v6d_id)


def main(args=None):
    rclpy.init(args=args)

    node = Node('test_detector_node')

    action_client = ActionClient(node, ProcessFrame, 'model_process_frame_action')

    for i in range(10000):

        goal_msg = ProcessFrame.Goal()
        # psg_public_msgs/Frame frame
        # unique_identifier_msgs/UUID x_uid
        goal_msg.frame.cache.id_int = int_id
        goal_msg.frame.cache.has_int_id = True
        goal_msg.frame.frame_num = i

        future = action_client.send_goal(goal_msg)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()