// this demonstrates how to create a simple video readout pipeline which consists of
// 1. video source node: read video file and publish frames
// 2. frame relay node: accept frames from video source node and publish them

#include <redoxi_example_cpp/_pch.hpp>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <json_struct/json_struct.h>

#include <redoxi_video_reader/VideoSourceFromUrl.hpp>
#include <yolo8_series/detection/Yolo8ObjectDetNode.hpp>
#include <redoxi_samples_nodes/sinks/DetectionRelayNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>


// pipeline:
// video_source_node -> detection_node -> detection_relay_node

//! Check if TEST_MODEL_DIR is defined, otherwise raise a compile error
#ifndef TEST_MODEL_DIR
#    error "TEST_MODEL_DIR must be defined. Please make sure it's properly set in CMakeLists.txt"
#endif


namespace rdx = redoxi_works;
namespace rdx_video = redoxi_works::video_readers;
namespace fs = std::filesystem;

namespace detection_relay_node_params
{
using Node_t = rdx::samples::DetectionRelayNode;
using InitConfig_t = Node_t::InitConfig_t;
using RuntimeConfig_t = Node_t::RuntimeConfig_t;
using NodeConfig_t = rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t>;

const std::string node_name = "detection_relay_node";
const std::string input_action_name = "input_detections";
const std::string output_topic_name = "relayed_detections";
const std::string output_visualization_topic_name = "relayed_visualization";
const std::chrono::milliseconds step_interval = std::chrono::milliseconds(10);

} // namespace detection_relay_node_params

namespace video_source_node_params
{
using Node_t = rdx_video::VideoSourceFromUrl;
using InitConfig_t = Node_t::InitConfig_t;
using RuntimeConfig_t = Node_t::RuntimeConfig_t;
using NodeConfig_t = rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t>;
using DeliveryPolicy_t = Node_t::DeliveryPolicy_t;

const std::string node_name = "video_source_node";
const bool allow_drop = true;
const std::chrono::milliseconds frame_interval = std::chrono::milliseconds(1000 / 50);
} // namespace video_source_node_params

namespace detection_node_params
{
using Node_t = rdx::model_nodes::yolo8::Yolo8ObjectDetNode;
using InitConfig_t = Node_t::InitConfig_t;
using RuntimeConfig_t = Node_t::RuntimeConfig_t;
using NodeConfig_t = rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t>;

const std::string node_name = "detection_node";
const std::string input_action_name = "input_frame";
const std::chrono::milliseconds detection_interval = std::chrono::milliseconds(10);
const fs::path model_path = fs::path(TEST_MODEL_DIR) / "yolov8s.onnx";
} // namespace detection_node_params


//! Check if TEST_DATA_DIR is defined, otherwise raise a compile error
#ifndef TEST_DATA_DIR
#    error "TEST_DATA_DIR must be defined. Please make sure it's properly set in CMakeLists.txt"
#endif
const fs::path test_data_dir = TEST_DATA_DIR;
const fs::path fn_video = "/soft/workspace/code/psf_ros2_ws/data/dancetrack/dancetrack-0039.mp4";

video_source_node_params::NodeConfig_t create_video_source_node_config();
detection_node_params::NodeConfig_t create_detection_node_config();
detection_relay_node_params::NodeConfig_t create_detection_relay_node_config();
int create_and_run_pipeline(int argc, char **argv);

int main(int argc, char **argv)
{
    return create_and_run_pipeline(argc, argv);
}

detection_relay_node_params::NodeConfig_t create_detection_relay_node_config()
{
    namespace configs = detection_relay_node_params;

    configs::NodeConfig_t config;
    config.init_config.input_port_config->set_action_name(configs::input_action_name);
    config.init_config.publish_detection_topic = configs::output_topic_name;
    config.init_config.publish_visualization_topic = configs::output_visualization_topic_name;

    // relay node will check its input at this interval,
    // if the check is faster than reader node's frame rate, all data will be relayed
    // otherwise, the upstream reader node may drop frames or retry, based on retry policy
    config.runtime_config.step_interval = configs::step_interval;

    return config;
}

video_source_node_params::NodeConfig_t create_video_source_node_config()
{
    namespace configs = video_source_node_params;
    namespace downstream_configs = detection_node_params;

    configs::NodeConfig_t config;
    config.init_config.video_url = fn_video.string();

    // add the frame relay node to the downstream
    configs::Node_t::DownstreamSpec_t ds_spec;

    // set the absolute action name of the downstream
    auto downstream_action_name = fmt::format("/{}/{}",
                                              downstream_configs::node_name, downstream_configs::input_action_name);
    ds_spec.set_action_name(downstream_action_name);

    // this is just an arbitrary name of the downstream for you to identify, unique among all downstreams
    // has nothing to do with downstream action name or node name
    ds_spec.set_name("send_to_detection");

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
    config.runtime_config.frame_interval = configs::frame_interval;

    // retry policy
    // the frame data will be first enqueued to the reader node's sending buffer, and then send to downstream
    // a frame may be dropped in the following cases:
    // 1. in enqueue stage, if sending buffer is full because sending is too slow
    // 2. in delivery stage, if downstream is not responsive or rejects the frame
    // so, we have 2 retry policies, one for enqueue stage, one for delivery stage
    if (configs::allow_drop) {
        // by default, the frame will not be dropped, and the sender retries as many times as possible
        // you can change the behavior by setting the drop strategy and retry policy as follows

        // enqueue stage policy
        auto &p_enqueue = config.runtime_config.frame_enqueue_policy;
        p_enqueue.set_drop_strategy(rdx::DropStrategy::DropAsNeeded);                            // allow drop in enqueue stage
        p_enqueue.get_retry_policy().set_number_of_retry(3);                                     // retry this number of times
        p_enqueue.get_retry_policy().set_wait_time_between_retry(std::chrono::milliseconds(10)); // wait this long between retries

        // delivery stage policy, it is std::optional by default, so you need to set it first
        config.runtime_config.frame_request_policy = configs::DeliveryPolicy_t();
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

detection_node_params::NodeConfig_t create_detection_node_config()
{
    namespace configs = detection_node_params;
    namespace downstream_configs = detection_relay_node_params;

    configs::NodeConfig_t config;

    // model path, in this case it is onnx model file
    // for this case, see Yolo8ModelConfig class, in Yolo8ModelTypes.hpp
    auto model_config = std::make_shared<configs::Node_t::InitConfig_t::ModelConfig_t>();
    model_config->set_string(rdx::inference::common_config_keys::ModelPath, configs::model_path.string());
    model_config->set_string(rdx::inference::common_config_keys::DeviceType, rdx::inference::common_device_types::CUDA);
    config.init_config.model_configs = {model_config};

    // image request config, this node will accept image processing requests from upstream
    // the image will go through detection model and then be sent to downstream
    // the upstream will only get an ACK after the image is accepted, without knowing when the result is ready
    config.init_config.image_request_config = configs::Node_t::InitConfig_t::ImageRequestConfig_t();
    config.init_config.image_request_config->input_port_config->set_action_name(configs::input_action_name);

    // create downstream as detection relay node, where detection result will be relayed to
    configs::Node_t::ByImageRequest::OutputDownstreamSpec_t ds_spec;
    auto downstream_action_name = fmt::format("/{}/{}",
                                              downstream_configs::node_name, downstream_configs::input_action_name);
    ds_spec.set_action_name(downstream_action_name);
    ds_spec.set_name("send_to_relay");
    config.init_config.image_request_config->output_port_config->set_downstream_specs({ds_spec});

    // optional
    config.runtime_config.step_interval = configs::detection_interval;
    config.runtime_config.enable_visualization = true;     // publish visualization topic?
    config.runtime_config.enable_performance_probe = true; // publish performance probe topic?

    // configure detector options, see ultralytics yolo8 documentation for more details
    config.runtime_config.model_output_config.conf_threshold = 0.25;
    config.runtime_config.model_output_config.iou_threshold = 0.45;

    // serialize the config to json string
    std::string config_as_json = JS::serializeStruct(config);
    spdlog::info("Detection node config: {}", config_as_json);

    return config;
}

//! Create and run the pipeline
//! Pipeline is: reader_node -> relay_node
//! reader_node: read video file and publish frames
//! relay_node: accept frames from reader_node and publish them
int create_and_run_pipeline(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    spdlog::info("Creating video source node config...");
    auto video_source_node_options = rclcpp::NodeOptions();
    {
        auto video_source_node_config = create_video_source_node_config();
        std::string config_as_json = JS::serializeStruct(video_source_node_config);
        video_source_node_options.parameter_overrides({{rdx::RosParams::ParamAsJsonString::MainKey, config_as_json}});
    }

    spdlog::info("Creating video source node...");
    auto video_source_node = std::make_shared<video_source_node_params::Node_t>(
        video_source_node_params::node_name,
        video_source_node_params::node_name,
        video_source_node_options);

    spdlog::info("Creating detection node config...");
    auto detection_node_options = rclcpp::NodeOptions();
    {
        auto detection_node_config = create_detection_node_config();
        std::string config_as_json = JS::serializeStruct(detection_node_config);
        detection_node_options.parameter_overrides({{rdx::RosParams::ParamAsJsonString::MainKey, config_as_json}});
    }

    spdlog::info("Creating detection node...");
    auto detection_node = std::make_shared<detection_node_params::Node_t>(
        detection_node_params::node_name,
        detection_node_params::node_name,
        detection_node_options);

    spdlog::info("Creating detection relay node config...");
    auto detection_relay_node_options = rclcpp::NodeOptions();
    {
        auto detection_relay_node_config = create_detection_relay_node_config();
        std::string config_as_json = JS::serializeStruct(detection_relay_node_config);
        detection_relay_node_options.parameter_overrides({{rdx::RosParams::ParamAsJsonString::MainKey, config_as_json}});
    }

    spdlog::info("Creating detection relay node...");
    auto detection_relay_node = std::make_shared<detection_relay_node_params::Node_t>(
        detection_relay_node_params::node_name,
        detection_relay_node_params::node_name,
        detection_relay_node_options);

    // configure all nodes
    spdlog::info("Configuring video source node...");
    video_source_node->configure();
    spdlog::info("Configuring detection node...");
    detection_node->configure();
    spdlog::info("Configuring detection relay node...");
    detection_relay_node->configure();

    // activate all nodes
    spdlog::info("Activating reader node...");
    video_source_node->activate();
    spdlog::info("Activating detection node...");
    detection_node->activate();
    spdlog::info("Activating detection relay node...");
    detection_relay_node->activate();

    // spin until shutdown
    spdlog::info("Spinning until shutdown...");
    //! Create an executor to spin both nodes
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(video_source_node->get_node_base_interface());
    executor.add_node(detection_node->get_node_base_interface());
    executor.add_node(detection_relay_node->get_node_base_interface());

    //! Spin until shutdown is called
    executor.spin();

    //! Deactivate nodes when shutting down
    spdlog::info("Deactivating detection relay node...");
    detection_relay_node->deactivate();
    spdlog::info("Deactivating detection node...");
    detection_node->deactivate();
    spdlog::info("Deactivating video source node...");
    video_source_node->deactivate();

    //! Cleanup nodes
    spdlog::info("Cleaning up detection relay node...");
    detection_relay_node->cleanup();
    spdlog::info("Cleaning up detection node...");
    detection_node->cleanup();
    spdlog::info("Cleaning up video source node...");
    video_source_node->cleanup();

    //! Shutdown ROS
    spdlog::info("Shutting down ROS...");
    rclcpp::shutdown();

    return 0;
}
