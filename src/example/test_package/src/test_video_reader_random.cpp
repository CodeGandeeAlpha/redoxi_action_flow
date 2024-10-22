#include <redoxi_video_reader/generators/RandomFrameVideoGenerator.hpp>
#include <spdlog/spdlog.h>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    auto init_config = std::make_shared<rdx::RandomFrameVideoGenerator::InitConfig_t>();
    auto runtime_config = std::make_shared<rdx::RandomFrameVideoGenerator::RuntimeConfig_t>();

    //! Initialize ROS 2
    spdlog::info("[MAIN] Initializing ROS 2");
    rclcpp::init(argc, argv);

    //! Create the RandomFrameVideoGenerator
    auto video_reader = std::make_shared<rdx::RandomFrameVideoGenerator>("random_frames");
    video_reader->get_logger().set_level(rclcpp::Logger::Level::Debug);

    spdlog::info("[MAIN] Initializing RandomFrameVideoGenerator");
    init_config->from_parameters(video_reader.get());
    for (auto &ds : init_config->downstreams) {
        spdlog::info("[MAIN] Downstream: {}", ds.first);
        ds.second->retry_strategy->set_max_number_of_retries(0);
    }

    runtime_config->frame_interval_ms = video_reader->get_parameter("frame_interval_ms").as_double();
    runtime_config->step_interval_ms = video_reader->get_parameter("step_interval_ms").as_double();

    video_reader->init(init_config, runtime_config);
    spdlog::info("[MAIN] Opening RandomFrameVideoGenerator");
    video_reader->open();
    video_reader->set_publish_image(true);
    spdlog::info("[MAIN] Starting RandomFrameVideoGenerator");
    video_reader->start();

    //! Spin the node
    spdlog::info("[MAIN] Spinning RandomFrameVideoGenerator");
    rclcpp::spin(video_reader);

    //! Stop and close the node
    spdlog::info("[MAIN] Stopping RandomFrameVideoGenerator");
    video_reader->stop();
    spdlog::info("[MAIN] Closing RandomFrameVideoGenerator");
    video_reader->close();

    //! Shutdown ROS 2
    spdlog::info("[MAIN] Shutting down ROS 2");
    rclcpp::shutdown();

    return 0;
}