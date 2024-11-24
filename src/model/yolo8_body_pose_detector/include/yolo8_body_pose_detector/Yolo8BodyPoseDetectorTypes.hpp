#pragma once

#include <optional>
#include <json_struct/json_struct.h>

#include <yolo8_body_pose_detector/yolo8_body_pose_detector.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionActionInputPort.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8PoseModel.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>

namespace redoxi_works::model_nodes::yolo8_body_pose_detector
{
using YoloModel_t = inference::yolo8::Yolo8PoseModel;
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

using DetectionActionInputPort = detection_ports::DetectionActionInputPort;

struct InitConfig : public common_nodes::StartStopNode::InitConfig_t {
    virtual ~InitConfig() = default;
    using ModelConfig_t = YoloModelConfig_t;
    using InputPortConfig_t = DetectionActionInputPort::InitConfig_t;

    // yolo8 model configurations, different model will work concurrently
    // if the same model config is pushed multiple times, they are regarded as replicas of the same model
    // and they will share the same model instance but using different inference inout data
    std::vector<ModelConfig_t::Ptr> model_configs;

    // use shared_ptr because the port asks for it
    std::shared_ptr<InputPortConfig_t> input_port_config = std::make_shared<InputPortConfig_t>();

    // debug topic
    std::string visualization_topic = "debug/visualization";

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
        JS_MEMBER(model_configs),
        JS_MEMBER(input_port_config),
        JS_MEMBER(visualization_topic));
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
        JS_MEMBER(_time_unit),
        JS_MEMBER(step_interval),
        JS_MEMBER(enable_blocking_mode),
        JS_MEMBER(model_output_config),
        JS_MEMBER(enable_visualization));
};
} // namespace redoxi_works::model_nodes::yolo8_body_pose_detector
