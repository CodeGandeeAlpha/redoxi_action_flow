#pragma once

#include <optional>
#include <json_struct/json_struct.h>

#include <yolo8_object_detector/yolo8_object_detector.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionRequestInputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionResponseOutputPort.hpp>
#include <redoxi_common_nodes/image_ports/AsyncImageInputPort.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8DetectionModel.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>


namespace redoxi_works::model_nodes::yolo8_object_detector
{

using YoloModel_t = inference::yolo8::Yolo8DetectionModel;
using YoloModelConfig_t = inference::yolo8::Yolo8ModelConfig;
using YoloModelOutputConfig_t = YoloModel_t::OutputConfig_t;

// a single inference resource, should be used by one single thread
struct InferenceResource {
    std::shared_ptr<YoloModel_t> model;
    inference::InferenceInOutData::Ptr inout_data;
    inference::yolo8::Yolo8ModelConfig::Ptr model_config;

    // the id of the replica
    int replica_id = 0;

    // index of this resource in the pool
    int index_in_pool = 0;
};

using DetectionRequestInputPort = detection_ports::request_response::DetectionRequestInputPort;
using ImageRequestInputPort = image_ports::AsyncImageInputPort;
using ImageRequestOutputPort = detection_ports::response_only::DetectionResponseOutputPort;

//! input/output ports config when you send detection request to this node
//! using this type of input, the node will process the detection request and send the result back to you
struct DetectionRequestConfig {
    using InputPort_t = DetectionRequestInputPort;
    std::shared_ptr<InputPort_t::InitConfig_t> input_port_config = std::make_shared<InputPort_t::InitConfig_t>();

    JS_OBJECT(JS_MEMBER(input_port_config));
};

//! input/output ports config when you send image to this node
//! using this type of input, the node will process the image but do not return detection result,
//! it will send the result to downstreams
struct ImageRequestConfig {
    using InputPort_t = ImageRequestInputPort;
    using OutputPort_t = ImageRequestOutputPort;
    std::shared_ptr<InputPort_t::InitConfig_t> input_port_config = std::make_shared<InputPort_t::InitConfig_t>();
    std::shared_ptr<OutputPort_t::InitConfig_t> output_port_config = std::make_shared<OutputPort_t::InitConfig_t>();

    // the delivery policy for the request enqueue request
    OutputPort_t::DeliveryPolicy_t output_enqueue_policy;

    JS_OBJECT(JS_MEMBER(input_port_config),
              JS_MEMBER(output_port_config),
              JS_MEMBER(output_enqueue_policy));
};

struct InitConfig : public common_nodes::StartStopNode::InitConfig_t {
    virtual ~InitConfig() = default;
    using ModelConfig_t = YoloModelConfig_t;
    using DetectionRequestConfig_t = DetectionRequestConfig;
    using ImageRequestConfig_t = ImageRequestConfig;

    // yolo8 model configurations, different model will work concurrently
    // if the same model config is pushed multiple times, they are regarded as replicas of the same model
    // and they will share the same model instance but using different inference inout data
    std::vector<ModelConfig_t::Ptr>
        model_configs;

    // detection request config
    std::optional<DetectionRequestConfig_t> detection_request_config;

    // image request config
    std::optional<ImageRequestConfig_t> image_request_config;

    // debug topic
    std::string publish_visualization_topic = "debug/visualization";

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
        JS_MEMBER(model_configs),
        JS_MEMBER(detection_request_config),
        JS_MEMBER(image_request_config),
        JS_MEMBER(publish_visualization_topic));
};

struct RuntimeConfig : public common_nodes::StartStopNode::RuntimeConfig_t {
    virtual ~RuntimeConfig() = default;
    using ModelOutputConfig_t = YoloModelOutputConfig_t;

    // enable blocking mode when reading data from input port
    bool enable_blocking_mode = false;

    // the default model output configurations
    ModelOutputConfig_t model_output_config;

    // publish to visualization topic?
    // only work if visualization_topic is set in init config
    bool enable_visualization = true;

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(common_nodes::StartStopNode::RuntimeConfig_t),
        JS_MEMBER(enable_blocking_mode),
        JS_MEMBER(model_output_config),
        JS_MEMBER(enable_visualization));
};


} // namespace redoxi_works::model_nodes::yolo8_object_detector
