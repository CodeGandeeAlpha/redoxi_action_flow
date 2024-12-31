#include <test_package/_pch.hpp>
// #include <yolo8_body_pose_detector/Yolo8BodyPoseDetectorNode.hpp>
#include <yolo8_series/bodypose/Yolo8BodyPoseNode.hpp>
#include <spdlog/spdlog.h>
#include <filesystem>
namespace fs = std::filesystem;

namespace rdx = redoxi_works;
namespace rdx_models = redoxi_works::model_nodes;
using RosNode_t = rdx_models::yolo8::Yolo8BodyPoseNode;

const fs::path model_path = "/soft/workspace/code/psf_ros2_ws/tmp/models/yolov8n-pose-640.onnx";

void print_config_jsons();

int main(int argc, char **argv)
{
    // print_config_jsons();
    // return 0;

    spdlog::info("Initializing ROS2...");
    //! Initialize ROS2
    rclcpp::init(argc, argv);

    spdlog::info("Creating detector node...");
    //! Create detector node
    auto node = std::make_shared<RosNode_t>("test_yolo_detector_node");

    spdlog::info("Creating and parsing init config...");
    auto init_config = std::make_shared<RosNode_t::InitConfig_t>();
    init_config->parse_from_node_parameters(init_config.get(), node.get());

    spdlog::info("Creating and parsing runtime config...");
    auto runtime_config = std::make_shared<RosNode_t::RuntimeConfig_t>();
    runtime_config->parse_from_node_parameters(runtime_config.get(), node.get());

    spdlog::info("Initializing node...");
    node->init(init_config, runtime_config);

    spdlog::info("Starting node...");
    node->start();

    spdlog::info("Spinning node...");
    //! Spin node
    rclcpp::spin(node);

    spdlog::info("Stopping node...");
    node->stop();

    spdlog::info("Shutting down ROS2...");
    //! Cleanup
    rclcpp::shutdown();
    return 0;
}

// DEFINE_NODE_CONFIG(RosNode_t::InitConfig_t, RosNode_t::RuntimeConfig_t);

void print_config_jsons()
{
    using ModelConfig_t = RosNode_t::InitConfig_t::ModelConfig_t;
    using InitConfig_t = RosNode_t::InitConfig_t;
    using RuntimeConfig_t = RosNode_t::RuntimeConfig_t;
    rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t> node_config;
    using DownstreamSpec_t = RosNode_t::ByImageRequest::OutputPort_t::DownstreamSpec_t;
    {
        RosNode_t::InitConfig_t init_config;

        // create model config
        ModelConfig_t::Ptr model_config = std::make_shared<ModelConfig_t>();
        model_config->set_string(ModelConfig_t::AcceptableKeys::ModelPath, model_path.string());
        init_config.model_configs.push_back(model_config);

        // create detection request config
        RosNode_t::InitConfig_t::DetectionRequestConfig_t detection_request_config;
        detection_request_config.input_port_config->set_action_name("in/detection_request");
        init_config.detection_request_config = detection_request_config;

        // create image request config
        RosNode_t::InitConfig_t::ImageRequestConfig_t image_request_config;
        image_request_config.input_port_config->set_action_name("in/image_request");
        DownstreamSpec_t ds;
        ds.set_action_name("/detection_sink/in/detection_response");
        image_request_config.output_port_config->set_downstream_specs({ds});
        init_config.image_request_config = image_request_config;
        node_config.init_config = init_config;
    }
    {
        RosNode_t::RuntimeConfig_t runtime_config;
        node_config.runtime_config = runtime_config;
    }
    std::string json_str = JS::serializeStruct(node_config);
    std::cout << "Node config: " << std::endl;
    std::cout << json_str << std::endl;
}