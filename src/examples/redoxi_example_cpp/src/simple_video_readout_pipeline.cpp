// this demonstrates how to create a simple video readout pipeline which consists of
// 1. video source node: read video file and publish frames
// 2. frame relay node: accept frames from video source node and publish them

#include <redoxi_example_cpp/_pch.hpp>
#include <filesystem>

#include <spdlog/spdlog.h>
#include <redoxi_video_reader/VideoSourceFromUrl.hpp>
#include <redoxi_samples_nodes/sinks/FrameRelayNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <json_struct/json_struct.h>


namespace rdx = redoxi_works;
namespace rdx_video = redoxi_works::video_readers;
namespace fs = std::filesystem;

namespace reader_node_params
{
using Node_t = rdx_video::VideoSourceFromUrl;
using InitConfig_t = Node_t::InitConfig_t;
using RuntimeConfig_t = Node_t::RuntimeConfig_t;
using NodeConfig_t = rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t>;
using DeliveryPolicy_t = Node_t::DeliveryPolicy_t;

const std::string node_name = "reader_node";
const std::string output_action_name = "out/action";
const bool allow_drop = true;
const int reader_fps = 10;
} // namespace reader_node_params

namespace relay_node_params
{
using Node_t = rdx::samples::FrameRelayNode;
using InitConfig_t = Node_t::InitConfig_t;
using RuntimeConfig_t = Node_t::RuntimeConfig_t;
using NodeConfig_t = rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t>;

const std::string node_name = "relay_node";
const std::string input_action_name = "input_frame";
const std::string output_topic_name = "relayed_frame";
const int relay_fps = 5;
} // namespace relay_node_params


//! Check if TEST_DATA_DIR is defined, otherwise raise a compile error
#ifndef TEST_DATA_DIR
#    error "TEST_DATA_DIR must be defined. Please make sure it's properly set in CMakeLists.txt"
#endif
const fs::path test_data_dir = TEST_DATA_DIR;
const fs::path fn_video = test_data_dir / "videos/dancetrack-0039.mp4";

reader_node_params::NodeConfig_t create_reader_node_config();
relay_node_params::NodeConfig_t create_relay_node_config();
int create_and_run_pipeline(int argc, char **argv);

int main(int argc, char **argv)
{
    return create_and_run_pipeline(argc, argv);
}

relay_node_params::NodeConfig_t create_relay_node_config()
{
    using namespace relay_node_params;

    NodeConfig_t config;
    config.init_config.input_port_config->set_action_name(input_action_name);
    config.init_config.publish_topic = output_topic_name;

    // relay node will check its input at this interval,
    // if the check is faster than reader node's frame rate, all data will be relayed
    // otherwise, the upstream reader node may drop frames or retry, based on retry policy
    config.runtime_config.step_interval = std::chrono::milliseconds(1000 / relay_fps);

    return config;
}

reader_node_params::NodeConfig_t create_reader_node_config()
{
    using namespace reader_node_params;

    NodeConfig_t config;
    config.init_config.video_url = fn_video.string();

    // add the frame relay node to the downstream
    Node_t::DownstreamSpec_t ds_spec;

    // set the absolute action name of the downstream
    auto downstream_action_name = fmt::format("/{}/{}", relay_node_params::node_name, relay_node_params::input_action_name);
    ds_spec.set_action_name(downstream_action_name);

    // this is just an arbitrary name of the downstream for you to identify, unique among all downstreams
    // has nothing to do with downstream action name or node name
    ds_spec.set_name("my_only_downstream");

    // add the downstream to the primary output port
    config.init_config.primary_output_spec->set_downstream_specs({ds_spec});

    // optional topic
    // source data (resized frame) will be published to this topic RELIABLY, frames will NOT be dropped
    // this is intended for external processing, you can subscribe to this topic to get the resized frame and handle them
    config.init_config.primary_output_spec->set_data_topic_for_source_data("data_msg/source_data");

    // optional topic
    // target data (resized frame) will be published to this topic UNRELIABLY, means frames can be dropped
    // this is intended for visualization, you can subscribe to this topic to visualize the resized frame
    config.init_config.primary_output_spec->set_visualization_topic_for_target_data("vis/target_data");

    // frame reading interval
    config.runtime_config.frame_interval = std::chrono::milliseconds(1000 / reader_fps);

    // retry policy
    // the frame data will be first enqueued to the reader node's sending buffer, and then send to downstream
    // a frame may be dropped in the following cases:
    // 1. in enqueue stage, if sending buffer is full because sending is too slow
    // 2. in delivery stage, if downstream is not responsive or rejects the frame
    // so, we have 2 retry policies, one for enqueue stage, one for delivery stage
    if (reader_node_params::allow_drop) {
        // by default, the frame will not be dropped, and the sender retries as many times as possible
        // you can change the behavior by setting the drop strategy and retry policy as follows

        // enqueue stage policy
        auto &p_enqueue = config.runtime_config.frame_enqueue_policy;
        p_enqueue.set_drop_strategy(rdx::DropStrategy::DropAsNeeded);                            // allow drop in enqueue stage
        p_enqueue.get_retry_policy().set_number_of_retry(3);                                     // retry this number of times
        p_enqueue.get_retry_policy().set_wait_time_between_retry(std::chrono::milliseconds(10)); // wait this long between retries

        // delivery stage policy, it is std::optional by default, so you need to set it first
        config.runtime_config.frame_request_policy = reader_node_params::DeliveryPolicy_t();
        auto &p_delivery = config.runtime_config.frame_request_policy.value();
        p_delivery.set_drop_strategy(rdx::DropStrategy::DropAsNeeded);                               // allow drop in delivery stage
        p_delivery.get_retry_policy().set_number_of_retry(3);                                        // retry this number of times
        p_delivery.get_retry_policy().set_wait_time_between_retry(std::chrono::milliseconds(10));    // wait this long between retries
        p_delivery.get_retry_policy().set_wait_time_retry_response(std::chrono::milliseconds(1000)); // wait this long for downstream to respond
    }

    // optional
    // output image size, the frame will be resized to this size
    config.runtime_config.output_image_size = cv::Size(640, -1);
    return config;
}

//! Create and run the pipeline
//! Pipeline is: reader_node -> relay_node
//! reader_node: read video file and publish frames
//! relay_node: accept frames from reader_node and publish them
int create_and_run_pipeline(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    spdlog::info("Creating reader node config...");
    auto reader_node_options = rclcpp::NodeOptions();
    {
        auto reader_node_config = create_reader_node_config();
        std::string config_as_json = JS::serializeStruct(reader_node_config);
        reader_node_options.parameter_overrides({{rdx::RosParams::ParamAsJsonString::MainKey, config_as_json}});
    }

    spdlog::info("Creating reader node...");
    auto reader_node = std::make_shared<reader_node_params::Node_t>(
        reader_node_params::node_name,
        reader_node_params::node_name,
        reader_node_options);

    spdlog::info("Creating relay node config...");
    auto relay_node_options = rclcpp::NodeOptions();
    {
        auto relay_node_config = create_relay_node_config();
        std::string config_as_json = JS::serializeStruct(relay_node_config);
        relay_node_options.parameter_overrides({{rdx::RosParams::ParamAsJsonString::MainKey, config_as_json}});
    }

    spdlog::info("Creating relay node...");
    auto relay_node = std::make_shared<relay_node_params::Node_t>(
        relay_node_params::node_name,
        relay_node_params::node_name,
        relay_node_options);

    // configure all nodes
    spdlog::info("Configuring reader node...");
    reader_node->configure();
    spdlog::info("Configuring relay node...");
    relay_node->configure();

    // activate all nodes
    spdlog::info("Activating reader node...");
    reader_node->activate();
    spdlog::info("Activating relay node...");
    relay_node->activate();

    // spin until shutdown
    spdlog::info("Spinning until shutdown...");
    //! Create an executor to spin both nodes
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(reader_node->get_node_base_interface());
    executor.add_node(relay_node->get_node_base_interface());

    //! Spin until shutdown is called
    executor.spin();

    //! Deactivate nodes when shutting down
    spdlog::info("Deactivating relay node...");
    relay_node->deactivate();
    spdlog::info("Deactivating reader node...");
    reader_node->deactivate();

    //! Cleanup nodes
    spdlog::info("Cleaning up relay node...");
    relay_node->cleanup();
    spdlog::info("Cleaning up reader node...");
    reader_node->cleanup();

    //! Shutdown ROS
    spdlog::info("Shutting down ROS...");
    rclcpp::shutdown();

    return 0;
}
