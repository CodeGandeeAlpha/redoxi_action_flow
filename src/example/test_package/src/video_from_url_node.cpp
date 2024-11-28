#include <redoxi_video_reader/VideoSourceFromUrl.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <spdlog/spdlog.h>

namespace rdx_video = redoxi_works::video_readers;
namespace rdx = redoxi_works;
// const char *fn_video = "/soft/workspace/code/psf_ros2_ws/data/20.22.6.214-2023-12-01-12-00-03_1400_1410.mp4";

void print_jsons();

int main(int argc, char **argv)
{
    //! Initialize ROS
    spdlog::info("Initializing ROS...");
    rclcpp::init(argc, argv);

    //! Create node with default options
    spdlog::info("Creating video source node...");
    auto node = std::make_shared<rdx_video::VideoSourceFromUrl>("video_source_from_url");

    //! Configure the node from parameters
    spdlog::info("Parsing initialization configuration from parameters...");
    auto init_config = std::make_shared<rdx_video::VideoSourceFromUrl::InitConfig_t>();
    init_config->parse_from_node_parameters(init_config.get(), node.get());
    // init_config->video_url = fn_video;

    //! Set runtime configuration from parameters
    spdlog::info("Parsing runtime configuration from parameters...");
    auto runtime_config = std::make_shared<rdx_video::VideoSourceFromUrl::RuntimeConfig_t>();
    runtime_config->parse_from_node_parameters(runtime_config.get(), node.get());

    spdlog::info("Initializing node...");
    node->init(init_config, runtime_config);

    //! Start the node
    spdlog::info("Starting node...");
    node->open();
    node->start();

    //! Spin until shutdown
    spdlog::info("Node started successfully. Spinning until shutdown...");
    rclcpp::spin(node);

    //! Cleanup
    node->stop();
    node->close();
    spdlog::info("Shutting down...");
    rclcpp::shutdown();
    return 0;
}

void print_jsons()
{
    using InitConfig_t = rdx_video::VideoSourceFromUrl::InitConfig_t;
    using RuntimeConfig_t = rdx_video::VideoSourceFromUrl::RuntimeConfig_t;
    using DownstreamSpec_t = rdx_video::VideoSourceFromUrl::DownstreamSpec_t;
    rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t> node_config;
    node_config.init_config.primary_output_spec->set_downstream_specs({DownstreamSpec_t()});
    auto json_string = JS::serializeStruct(node_config);
    std::cout << json_string << std::endl;
}