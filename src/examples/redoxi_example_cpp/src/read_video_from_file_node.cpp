// this demonstrates how to read video file using redoxi_video_reader
// the frames will be transferred to downstream node by action

#include <redoxi_example_cpp/_pch.hpp>

#include <spdlog/spdlog.h>
#include <redoxi_video_reader/VideoSourceFromUrl.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <json_struct/json_struct.h>

#include <filesystem>
namespace rdx = redoxi_works;
namespace rdx_video = redoxi_works::video_readers;
namespace fs = std::filesystem;

using InitConfig_t = rdx_video::VideoSourceFromUrl::InitConfig_t;
using RuntimeConfig_t = rdx_video::VideoSourceFromUrl::RuntimeConfig_t;
using NodeConfig_t = rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t>;

//! Check if TEST_DATA_DIR is defined, otherwise raise a compile error
#ifndef TEST_DATA_DIR
#    error "TEST_DATA_DIR must be defined. Please make sure it's properly set in CMakeLists.txt"
#endif
const fs::path test_data_dir = TEST_DATA_DIR;
const fs::path fn_video = "/soft/workspace/code/psf_ros2_ws/data/dancetrack/dancetrack-0039.mp4";

int create_ordinary_node(int argc, char **argv);
int create_lifecycle_node(int argc, char **argv);

//! The node is stateful, it goes through ROS lifecycle node states.
//! see https://design.ros2.org/articles/node_lifecycle.html


//! Create initialization configuration, which is used to transit the node from UNCONFIGURED to INACTIVE state
InitConfig_t create_init_config();

//! Create runtime configuration, which is used to transit the node from INACTIVE to ACTIVE state
RuntimeConfig_t create_runtime_config();

//! Create the node configuration, including init and runtime configs
NodeConfig_t create_node_config();

int main(int argc, char **argv)
{
    // return lifecycle_style(argc, argv);
    return create_lifecycle_node(argc, argv);
}

InitConfig_t create_init_config()
{
    //! Create a new initialization configuration
    InitConfig_t init_config;

    //! Set the video URL to the test video file
    init_config.video_url = fn_video;

    // optional topic
    // source data (resized frame) will be published to this topic RELIABLY, frames will NOT be dropped
    // this is intended for external processing, you can subscribe to this topic to get the resized frame and handle them
    init_config.primary_output_spec->set_data_topic_for_source_data("data_msg/source_data");

    // optional topic
    // target data (resized frame) will be published to this topic UNRELIABLY, means frames can be dropped
    // this is intended for visualization, you can subscribe to this topic to visualize the resized frame
    init_config.primary_output_spec->set_visualization_topic_for_target_data("visualize/target_data");

    // config can be serialized to/from json string
    // this is useful for logging and debugging
    // for more information, please refer to json_struct documentation
    // std::string js_config = JS::serializeStruct(init_config);
    // spdlog::info("Init config JSON: {}", js_config);

    return init_config;
}

RuntimeConfig_t create_runtime_config()
{
    //! Create a new runtime configuration
    RuntimeConfig_t runtime_config;

    // read video at this fps
    int fps = 25;
    runtime_config.step_interval = std::chrono::milliseconds(1000 / fps);

    // optional
    // resize the frame to this size, -1 means keep the original aspect ratio and compute that dimension
    runtime_config.output_image_size = cv::Size(1024, -1);

    // config can be serialized to/from json string
    // this is useful for logging and debugging
    // for more information, please refer to json_struct documentation
    // std::string js_config = JS::serializeStruct(runtime_config);
    // spdlog::info("Runtime config JSON: {}", js_config);
    return runtime_config;
}

NodeConfig_t create_node_config()
{
    NodeConfig_t node_config;
    node_config.init_config = create_init_config();
    node_config.runtime_config = create_runtime_config();
    return node_config;
}

int create_lifecycle_node(int argc, char **argv)
{
    //! Initialize ROS
    spdlog::info("Initializing ROS...");
    rclcpp::init(argc, argv);

    //! Create node configuration, and serialize it to json string, pass it to node as ros parameter
    spdlog::info("Creating node configuration...");
    auto node_config = create_node_config();
    std::string config_as_json = JS::serializeStruct(node_config);
    spdlog::info("Node config JSON: {}", config_as_json);

    rclcpp::NodeOptions options;
    options.parameter_overrides({{rdx::RosParams::ParamAsJsonString::MainKey, config_as_json}});

    //! Create node with given options
    spdlog::info("Creating video source node...");
    auto node = std::make_shared<rdx_video::VideoSourceFromUrl>("my_video_source", options);

    //! Configure the node, lifecycle node needs this to transit from UNCONFIGURED to INACTIVE
    spdlog::info("configure(): Configuring node...");
    node->configure();

    //! Start the node, lifecycle node needs this to transit from INACTIVE to ACTIVE
    spdlog::info("activate(): Activating node...");
    node->activate();

    // Now you can use ros2 commands to see what topics are published by this node
    // Start rosboard to checkout the topics
    // Note that those topics are used for pub/sub processing or visualization, not for action processing
    // For action processing, you need to have downstream nodes to accept the action requests

    //! Spin until shutdown
    spdlog::info("Node started successfully. Spinning until shutdown...");
    rclcpp::spin(node->get_node_base_interface());

    //! Deactivate the node, lifecycle node needs this to transit from ACTIVE to INACTIVE
    spdlog::info("deactivate(): Deactivating node...");
    node->deactivate();

    //! Cleanup, lifecycle node needs this to transit from INACTIVE to UNCONFIGURED
    spdlog::info("cleanup(): Cleaning up node...");
    node->cleanup();

    //! Shutdown ROS
    spdlog::info("Shutting down...");
    rclcpp::shutdown();
    return 0;
}

//! The states also have different names in Redoxi framework,
//! BEFORE_INIT -> [init()] -> CLOSED -> [open()] -> OPENED -> [start()] -> STARTED -> [stop()] -> STOPPED -> [close()] -> CLOSED
int create_ordinary_node(int argc, char **argv)
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