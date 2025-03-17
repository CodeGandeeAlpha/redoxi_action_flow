// this demonstrates how to read video file using redoxi_video_reader
// the frames will be transferred to downstream node by action

#include <redoxi_example_cpp/_pch.hpp>

#include <spdlog/spdlog.h>
#include <redoxi_video_reader/VideoSourceFromUrl.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

#include <filesystem>
namespace rdx_video = redoxi_works::video_readers;
namespace rdx = redoxi_works;
namespace fs = std::filesystem;

//! Check if TEST_DATA_DIR is defined, otherwise raise a compile error
#ifndef TEST_DATA_DIR
#    error "TEST_DATA_DIR must be defined. Please make sure it's properly set in CMakeLists.txt"
#endif
const fs::path test_data_dir = TEST_DATA_DIR;
const fs::path fn_video = test_data_dir / "dancetrack/dancetrack-0039.mp4";

void print_default_json_config();
int lifecycle_style(int argc, char **argv);
int manual_create_node_minimal(int argc, char **argv);
int manual_create_node_advanced(int argc, char **argv);


int main(int argc, char **argv)
{
    // return lifecycle_style(argc, argv);
    return manual_create_node_minimal(argc, argv);
}

int lifecycle_style(int argc, char **argv)
{
    //! Initialize ROS
    spdlog::info("Initializing ROS...");
    rclcpp::init(argc, argv);

    //! Create node with default options
    spdlog::info("Creating video source node...");
    auto node = std::make_shared<rdx_video::VideoSourceFromUrl>("video_source_from_url");

    spdlog::info("configure(): Initializing node with launch options...");
    node->configure();

    //! Start the node
    spdlog::info("activate(): Starting node...");
    node->activate();

    //! Spin until shutdown
    spdlog::info("Node started successfully. Spinning until shutdown...");
    rclcpp::spin(node->get_node_base_interface());

    //! Cleanup
    // node->deactivate();
    // node->cleanup();
    spdlog::info("Shutting down...");
    rclcpp::shutdown();
    return 0;
}

int manual_create_node_minimal(int argc, char **argv)
{
    //! Initialize ROS
    spdlog::info("Initializing ROS...");
    rclcpp::init(argc, argv);

    //! Create node with default options
    spdlog::info("Creating video source node...");
    auto node = std::make_shared<rdx_video::VideoSourceFromUrl>("video_source_from_url");

    //! Configure the node from parameters
    spdlog::info("Creating initialization configuration...");
    auto init_config = std::make_shared<rdx_video::VideoSourceFromUrl::InitConfig_t>();
    init_config->video_url = fn_video;

    // optional topic
    // source data (resized frame) will be published to this topic RELIABLY, frames will NOT be dropped
    // this is intended for external processing, you can subscribe to this topic to get the resized frame and handle them
    init_config->primary_output_spec->set_data_topic_for_source_data("data_msg/source_data");

    // optional topic
    // target data (resized frame) will be published to this topic UNRELIABLY, means frames can be dropped
    // this is intended for visualization, you can subscribe to this topic to visualize the resized frame
    init_config->primary_output_spec->set_visualization_topic_for_target_data("visualize/target_data");

    //! Set runtime configuration from parameters
    spdlog::info("Creating runtime configuration...");
    auto runtime_config = std::make_shared<rdx_video::VideoSourceFromUrl::RuntimeConfig_t>();

    // read video at this fps
    int fps = 100;
    runtime_config->step_interval = std::chrono::milliseconds(1000 / fps);

    // resize the frame to this size, -1 means keep the original aspect ratio and compute that dimension
    // this is optional, if not set, the frame will be published without resizing
    runtime_config->output_image_size = cv::Size(512, -1);

    spdlog::info("Initializing node...");
    node->init(init_config, runtime_config);

    //! Start the node
    spdlog::info("Starting node...");
    node->open();
    node->start();

    //! Spin until shutdown
    spdlog::info("Node started successfully. Spinning until shutdown...");
    rclcpp::spin(node->get_node_base_interface());

    //! Cleanup
    node->stop();
    node->close();
    spdlog::info("Shutting down...");
    rclcpp::shutdown();
    return 0;
}

int manual_create_node_advanced(int argc, char **argv)
{
    //! Initialize ROS
    spdlog::info("Initializing ROS...");
    rclcpp::init(argc, argv);

    //! Create node with default options
    spdlog::info("Creating video source node...");
    auto node = std::make_shared<rdx_video::VideoSourceFromUrl>("video_source_from_url");

    //! Configure the node from parameters
    spdlog::info("Creating initialization configuration...");
    auto init_config = std::make_shared<rdx_video::VideoSourceFromUrl::InitConfig_t>();
    init_config->video_url = fn_video;

    // optional topic
    // source data (original frame) will be published to this topic without loss
    // if you want to process the data using pub/sub, you can subscribe to this topic
    init_config->primary_output_spec->set_data_topic_for_source_data("data_msg/source_data");

    // optional topic
    // target data (resized frame) will be published to this topic without loss
    // if you want to process the data using pub/sub, you can subscribe to this topic
    init_config->primary_output_spec->set_data_topic_for_target_data("data_msg/target_data");

    // optional topic
    // source data (original frame) will be published to this topic UNRELIABLY, means frames can be dropped
    // this is intended for visualization, you can subscribe to this topic to visualize the original frame
    init_config->primary_output_spec->set_probe_topic_for_source_data("probe/source_data");

    // optional topic
    // target data (resized frame) will be published to this topic UNRELIABLY, means frames can be dropped
    // this is intended for visualization, you can subscribe to this topic to visualize the resized frame
    init_config->primary_output_spec->set_probe_topic_for_target_data("probe/target_data");

    //! Set runtime configuration from parameters
    spdlog::info("Creating runtime configuration...");
    auto runtime_config = std::make_shared<rdx_video::VideoSourceFromUrl::RuntimeConfig_t>();

    // read video at 10 fps
    int fps = 10;
    runtime_config->step_interval = std::chrono::milliseconds(1000 / fps);

    spdlog::info("Initializing node...");
    node->init(init_config, runtime_config);

    //! Start the node
    spdlog::info("Starting node...");
    node->open();
    node->start();

    //! Spin until shutdown
    spdlog::info("Node started successfully. Spinning until shutdown...");
    rclcpp::spin(node->get_node_base_interface());

    //! Cleanup
    node->stop();
    node->close();
    spdlog::info("Shutting down...");
    rclcpp::shutdown();
    return 0;
}

void print_default_json_config()
{
    using InitConfig_t = rdx_video::VideoSourceFromUrl::InitConfig_t;
    using RuntimeConfig_t = rdx_video::VideoSourceFromUrl::RuntimeConfig_t;
    using DownstreamSpec_t = rdx_video::VideoSourceFromUrl::DownstreamSpec_t;
    rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t> node_config;
    node_config.init_config.primary_output_spec->set_downstream_specs({DownstreamSpec_t()});
    auto json_string = JS::serializeStruct(node_config);
    std::cout << json_string << std::endl;
}