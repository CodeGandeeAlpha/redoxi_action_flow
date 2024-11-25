#include <redoxi_samples_nodes/generators/RandomFrameVideoGenerator.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //! Create a shared pointer for the RandomFrameVideoGenerator node
    auto video_generator = std::make_shared<rdx::RandomFrameVideoGenerator>("v2_random_video_source");

    //! Create and initialize the node's configuration
    auto init_config = std::make_shared<rdx::RandomFrameVideoGenerator::InitConfig_t>();
    auto runtime_config = std::make_shared<rdx::RandomFrameVideoGenerator::RuntimeConfig_t>();

    init_config->parse_from_node_parameters(init_config.get(), video_generator.get());
    runtime_config->parse_from_node_parameters(runtime_config.get(), video_generator.get());

    //! Initialize the node with the configuration
    video_generator->init(init_config, runtime_config);

    //! Open the node to prepare it for operation
    video_generator->open();

    //! Start the node to begin generating random frames
    video_generator->start();

    //! Spin the node to process callbacks and keep it running
    rclcpp::spin(video_generator);

    //! Stop and close the node before shutting down
    video_generator->stop();
    video_generator->close();

    //! Shutdown ROS 2
    rclcpp::shutdown();
}