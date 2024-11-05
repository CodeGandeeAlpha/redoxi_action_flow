#include <redoxi_samples_nodes/sinks/FrameRelayPublisher.hpp>
#include <spdlog/spdlog.h>

namespace rdx = redoxi_works;
using rdx::RDX_LOG_INFO;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rdx::FrameRelayPublisher>("video_sink");

    RDX_LOG_INFO(node.get(), __func__, "{}", "Starting FrameRelayPublisher");

    // create and initialize the node
    auto init_config = std::make_shared<rdx::FrameRelayPublisher::InitConfig_t>();
    init_config->from_parameters(node.get());
    // init_config->use_async = true;
    RDX_LOG_INFO(node.get(), __func__, "{}", "Initializing FrameRelayPublisher");
    node->init(init_config);

    RDX_LOG_INFO(node.get(), __func__, "{}", "Spinning FrameRelayPublisher");
    rclcpp::spin(node);

    RDX_LOG_INFO(node.get(), __func__, "{}", "Shutting down FrameRelayPublisher");
    rclcpp::shutdown();

    return 0;
}
