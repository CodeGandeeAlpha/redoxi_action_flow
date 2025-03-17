import numpy as np
import rclpy
import rclpy.logging
from rclpy.node import Node
from redoxi_public_msgs.action import ProcessFrame
from unique_identifier_msgs.msg import UUID as RosUUID
from sensor_msgs.msg import Image as RosImage
import rclpy.logging as logging
import cv2

logf = logging.get_logger("mylog")


# subscribe to goal message from a publisher
class ActionSub(Node):
    def __init__(self):
        super().__init__("action_sub")

        self.m_sub = self.create_subscription(
            ProcessFrame.Goal,
            "/video_source/data_out/target_data",
            self.on_goal,
            10,
        )

        self.m_sub_relayed_image = self.create_subscription(
            RosImage,
            "/frame_relay/out/relayed_frame",
            self.on_relayed_image,
            10,
        )
        # all_topics = self.get_topic_names_and_types()
        # for topic_name, topic_type in all_topics:
        #     logf.info(f"Topic: {topic_name}, Type: {topic_type}")

    def on_goal(self, msg: ProcessFrame.Goal):
        logf.info("Received goal message")
        uuid_str = "".join(format(x, "02x") for x in msg.x_uid.uuid)
        img: RosImage = msg.frame_bundle.primary_frame.raw_image
        img_array = np.frombuffer(img.data, dtype=np.uint8).reshape(
            (img.height, img.width, -1)
        )
        cv2.namedWindow("goal image", cv2.WINDOW_NORMAL)
        cv2.imshow("goal image", img_array[:, :, ::-1])
        cv2.waitKey(5)
        logf.info(f"msg uuid: {uuid_str}")
        logf.info(f"img shape: {img_array.shape}")

    def on_relayed_image(self, msg: RosImage):
        logf.info("Received relayed image")
        img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, -1)
        )
        cv2.namedWindow("relayed_img", cv2.WINDOW_NORMAL)
        cv2.imshow("relayed_img", img_array[:, :, ::-1])
        cv2.waitKey(5)


def main(args=None):
    rclpy.init(args=args)
    logf.info("Initialized ROS2")
    sub = ActionSub()
    logf.info("Created action subscriber")
    rclpy.spin(sub)
    logf.info("Spinning action subscriber")
    rclpy.shutdown()
    logf.info("Shutting down ROS2")
    rclpy.logging.get_logger()


if __name__ == "__main__":
    main()
