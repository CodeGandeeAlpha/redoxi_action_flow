#include <redoxi_video_reader/generators/RandomFrameVideoReader.hpp>
#include <spdlog/spdlog.h>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    auto init_config = std::make_shared<rdx::RandomFrameVideoReader::InitConfig_t>();
    auto runtime_config = std::make_shared<rdx::RandomFrameVideoReader::RuntimeConfig_t>();

    //! Initialize ROS 2
    spdlog::info("[MAIN] Initializing ROS 2");
    rclcpp::init(argc, argv);

    //! Create the RandomFrameVideoReader
    auto video_reader = std::make_shared<rdx::RandomFrameVideoReader>("random_frames");
    video_reader->get_logger().set_level(rclcpp::Logger::Level::Debug);

    spdlog::info("[MAIN] Initializing RandomFrameVideoReader");
    video_reader->init(init_config, runtime_config);
    spdlog::info("[MAIN] Opening RandomFrameVideoReader");
    video_reader->open();
    video_reader->set_publish_image(true);
    spdlog::info("[MAIN] Starting RandomFrameVideoReader");
    video_reader->start();

    //! Spin the node
    spdlog::info("[MAIN] Spinning RandomFrameVideoReader");
    rclcpp::spin(video_reader);

    //! Stop and close the node
    spdlog::info("[MAIN] Stopping RandomFrameVideoReader");
    video_reader->stop();
    spdlog::info("[MAIN] Closing RandomFrameVideoReader");
    video_reader->close();

    //! Shutdown ROS 2
    spdlog::info("[MAIN] Shutting down ROS 2");
    rclcpp::shutdown();

    return 0;
}