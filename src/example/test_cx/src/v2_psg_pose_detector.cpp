#include <psg_pose_detector/Pipeline.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //! Create a node options object
    rclcpp::NodeOptions options;

    //! Create a FrameRelayNode with a specific name and options
    auto psg_pose_detector_node = std::make_shared<rdx::PSGPoseDetectorNode>("psg_pose_detector_node", options);

    //! Initialize the node with default configuration
    auto init_config = std::make_shared<rdx::PSGPoseDetectorNode::InitConfig_t>();
    init_config->from_parameters(psg_pose_detector_node.get());
    auto runtime_config = std::make_shared<rdx::PSGPoseDetectorNode::RuntimeConfig_t>();
    runtime_config->from_parameters(psg_pose_detector_node.get());
    psg_pose_detector_node->init(init_config, runtime_config);

    //! Start the node
    psg_pose_detector_node->start();

    //! Keep the node running
    rclcpp::spin(psg_pose_detector_node);

    //! Stop the node before shutdown
    psg_pose_detector_node->stop();

    //! Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
