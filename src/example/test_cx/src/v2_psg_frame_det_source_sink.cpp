#include <psg_frame_det_source_sink/PSGFrameDetSourceSink.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //! Create a node options object
    rclcpp::NodeOptions options;

    //! Create a FrameRelayNode with a specific name and options
    auto psg_frame_det_source_sink_node = std::make_shared<rdx::PSGFrameDetSourceSink>("psg_frame_det_source_sink_node", options);

    //! Initialize the node with default configuration
    auto init_config = std::make_shared<rdx::PSGFrameDetSourceSink::InitConfig_t>();
    init_config->from_parameters(psg_frame_det_source_sink_node.get());
    auto runtime_config = std::make_shared<rdx::PSGFrameDetSourceSink::RuntimeConfig_t>();
    runtime_config->from_parameters(psg_frame_det_source_sink_node.get());
    psg_frame_det_source_sink_node->init(init_config, runtime_config);

    //! Start the node
    psg_frame_det_source_sink_node->start();

    //! Keep the node running
    rclcpp::spin(psg_frame_det_source_sink_node);

    //! Stop the node before shutdown
    psg_frame_det_source_sink_node->stop();

    //! Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
