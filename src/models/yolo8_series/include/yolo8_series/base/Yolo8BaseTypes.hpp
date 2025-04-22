#pragma once

// required by json_struct to use std::optional and std::unordered_map
// #ifndef JS_STD_OPTIONAL
// #    define JS_STD_OPTIONAL
// #endif

// #ifndef JS_STD_UNORDERED_MAP
// #    define JS_STD_UNORDERED_MAP
// #endif

#include <optional>
#include <json_struct/json_struct.h>
#include <yolo8_series/yolo8_series.hpp>

#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionRequestInputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionResponseOutputPort.hpp>
#include <redoxi_common_nodes/image_ports/AsyncImageInputPort.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelBase.hpp>
// #include <redoxi_dnn_models/yolo8/Yolo8PoseModel.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>

namespace redoxi_works::model_nodes::yolo8
{

//! Concept for a YOLO model
template <typename T>
concept YoloModelConcept = requires(T model)
{
    requires std::derived_from<T, inference::yolo8::Yolo8ModelBase>;

    typename T::InitConfig_t;
    requires std::derived_from<typename T::InitConfig_t, inference::yolo8::Yolo8ModelConfig>;

    typename T::OutputConfig_t;
    requires std::derived_from<typename T::OutputConfig_t, inference::yolo8::PostprocessorConfig>;

    typename T::SingleImageOutput_t;
    requires std::derived_from<typename T::SingleImageOutput_t, inference::detection::types::SingleImageOutput>;

    typename T::DetectedObject_t;
    requires std::derived_from<typename T::DetectedObject_t, inference::detection::types::DetectedObject>;

    typename T::Keypoint_t;
    requires std::derived_from<typename T::Keypoint_t, inference::detection::types::Keypoint>;
};

// a single inference resource, should be used by one single thread
template <YoloModelConcept TModel>
struct InferenceResource {
    using Model_t = TModel;
    using ModelConfig_t = typename Model_t::InitConfig_t;

    std::shared_ptr<Model_t> model;
    inference::InferenceInOutData::Ptr inout_data;
    std::shared_ptr<ModelConfig_t> model_config;

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

template <YoloModelConcept TModel>
struct InitConfig : public common_nodes::StartStopNode::InitConfig_t {
    virtual ~InitConfig() = default;
    using Model_t = TModel;
    using ModelConfig_t = typename Model_t::InitConfig_t;
    using DetectionRequestConfig_t = DetectionRequestConfig;
    using ImageRequestConfig_t = ImageRequestConfig;

    // yolo8 model configurations, different model will work concurrently
    // if the same model config is pushed multiple times, they are regarded as replicas of the same model
    // and they will share the same model instance but using different inference inout data
    std::vector<std::shared_ptr<ModelConfig_t>> model_configs;

    // detection request config
    std::optional<DetectionRequestConfig_t> detection_request_config;

    // image request config
    std::optional<ImageRequestConfig_t> image_request_config;

    // debug topic
    std::string publish_visualization_topic = "debug/visualization";

    // performance probe topic
    std::string publish_probe_detection_done_topic = "probe/detection_done";

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
        JS_MEMBER(model_configs),
        JS_MEMBER(detection_request_config),
        JS_MEMBER(image_request_config),
        JS_MEMBER(publish_visualization_topic),
        JS_MEMBER(publish_probe_detection_done_topic));
};

template <YoloModelConcept TModel>
struct RuntimeConfig : public common_nodes::StartStopNode::RuntimeConfig_t {
    using Model_t = TModel;
    using ModelOutputConfig_t = typename Model_t::OutputConfig_t;

    virtual ~RuntimeConfig() = default;

    // enable blocking mode when reading data from input port
    bool enable_blocking_mode = false;

    // the default model output configurations
    ModelOutputConfig_t model_output_config;

    // publish to visualization topic?
    // only work if visualization_topic is set in init config
    bool enable_visualization = true;

    // enable performance probe?
    bool enable_performance_probe = false;

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(common_nodes::StartStopNode::RuntimeConfig_t),
        JS_MEMBER(enable_blocking_mode),
        JS_MEMBER(model_output_config),
        JS_MEMBER(enable_visualization),
        JS_MEMBER(enable_performance_probe));
};
} // namespace redoxi_works::model_nodes::yolo8
