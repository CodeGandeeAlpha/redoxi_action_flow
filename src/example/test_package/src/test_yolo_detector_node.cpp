#include <yolo8_body_pose_detector/Yolo8BodyPoseDetector.hpp>

namespace rdx_models = redoxi_works::model_nodes;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rdx_models::Yolo8BodyPoseDetector>("test_yolo_detector_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}