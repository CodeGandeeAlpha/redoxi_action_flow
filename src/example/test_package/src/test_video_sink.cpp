#include <redoxi_video_reader/sinks/FrameRelayPublisher.hpp>
#include <spdlog/spdlog.h>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rdx::FrameRelayPublisher>("video_sink");

    spdlog::info("[SINK] Creating FrameRelayPublisher");
    // create and initialize the node
    auto init_config = std::make_shared<rdx::FrameRelayPublisher::InitConfig_t>();
    node->init(init_config);

    spdlog::info("[SINK] Spinning FrameRelayPublisher");
    rclcpp::spin(node);

    spdlog::info("[SINK] Shutting down FrameRelayPublisher");
    rclcpp::shutdown();

    spdlog::info("[SINK] Closed");
    return 0;
}
