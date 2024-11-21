#pragma once

#include <optional>
#include <json_struct/json_struct.h>

#include <yolo8_body_pose_detector/yolo8_body_pose_detector.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionActionInputPort.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8PoseModel.hpp>

namespace redoxi_works::model_nodes
{
class Yolo8BodyPoseDetector;
}

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

struct InitConfig {
    virtual ~InitConfig() = default;
    using ModelConfig_t = YoloModelConfig_t;
    using InputPortConfig_t = DetectionActionInputPort::InitConfig_t;

    // yolo8 model configurations, different model will work concurrently
    // if the same model config is pushed multiple times, they are regarded as replicas of the same model
    // and they will share the same model instance but using different inference inout data
    std::vector<ModelConfig_t::Ptr> model_configs;

    // use shared_ptr because the port asks for it
    std::shared_ptr<InputPortConfig_t> input_port_config = std::make_shared<InputPortConfig_t>();

    virtual void from_parameters(const Yolo8BodyPoseDetector *node);
    JS_OBJECT(JS_MEMBER(model_configs),
              JS_MEMBER(input_port_config));
};

struct RuntimeConfig {
    using TimeUnit = DetectionActionInputPort::TimeUnit_t;
    virtual ~RuntimeConfig() = default;
    using ModelOutputConfig_t = YoloModelOutputConfig_t;

    // nothing yet
    inline static const TimeUnit DefaultStepInterval{std::chrono::milliseconds(10)};

    // for annotation only, do not change it
    std::string _time_unit = _get_time_unit_name<TimeUnit>();

    // step interval for the action server
    TimeUnit step_interval = DefaultStepInterval;

    // enable blocking mode when reading data from input port
    bool enable_blocking_mode = false;

    // the default model output configurations
    ModelOutputConfig_t model_output_config;

    virtual void from_parameters(const Yolo8BodyPoseDetector *node);
    JS_OBJECT(JS_MEMBER(_time_unit),
              JS_MEMBER(step_interval),
              JS_MEMBER(enable_blocking_mode),
              JS_MEMBER(member_model_output_config));
};
} // namespace redoxi_works::model_nodes::yolo8_body_pose_detector
