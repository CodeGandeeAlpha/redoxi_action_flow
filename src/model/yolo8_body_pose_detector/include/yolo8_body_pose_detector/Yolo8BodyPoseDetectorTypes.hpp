#pragma once

#include <optional>
#include <json_struct/json_struct.h>

#include <yolo8_body_pose_detector/yolo8_body_pose_detector.hpp>
#include <redoxi_public_msgs/action/process_detections_by_frame.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8PoseModel.hpp>

namespace redoxi_works::model_nodes
{
class Yolo8BodyPoseDetector;
}

namespace redoxi_works::model_nodes::yolo8_body_pose_detector
{

// a single inference resource, should be used by one single thread
struct InferenceResource {
    std::shared_ptr<inference::yolo8::Yolo8PoseModel> model;
    inference::InferenceInOutData::Ptr inout_data;
    inference::yolo8::Yolo8ModelConfig::Ptr model_config;

    // the id of the replica
    int replica_id = 0;
};

using TimeUnit = DefaultTimeUnit_t;
using DetectionActionType = redoxi_public_msgs::action::ProcessDetectionsByFrame;
using DetectionActionDataTrait = RedoxiActionDataTrait<DetectionActionType>;
static_assert(RedoxiActionConcept<DetectionActionType>, "DetectionActionType must satisfy RedoxiActionConcept");

// the source data for the detection action input port
// should be bound to a single inference resource
class DetectionSourceData : public input_port_types::DefaultReceiveSourceData<DetectionActionType>
{
  public:
    std::optional<InferenceResource> model_resource;
};


struct DetectionActionInputPortSpec {
    using ActionType_t = DetectionActionType;
    using ActionGoal_t = ActionType_t::Goal;
    using ActionDataTrait_t = DetectionActionDataTrait;
    using TimeUnit_t = TimeUnit;
    using ReceiveSourceData_t = DetectionSourceData;
    using InitConfig_t = input_port_types::DefaultInitConfig<TimeUnit>;
};
static_assert(input_port_types::AsyncActionInputPortSpecConcept<DetectionActionInputPortSpec>,
              "DetectionActionInputPortSpec must satisfy AsyncActionInputPortSpecConcept");

using DetectionActionInputPort = AsyncActionInputPort<DetectionActionInputPortSpec>;

struct InitConfig {
    virtual ~InitConfig() = default;
    using ModelConfig_t = inference::yolo8::Yolo8ModelConfig;
    using InputPortConfig_t = DetectionActionInputPortSpec::InitConfig_t;

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
    // nothing yet
    inline static const TimeUnit DefaultStepInterval{std::chrono::milliseconds(10)};

    // for annotation only, do not change it
    std::string _time_unit = _get_time_unit_name<TimeUnit>();

    // step interval for the action server
    TimeUnit step_interval = DefaultStepInterval;

    // enable blocking mode when reading data from input port
    bool enable_blocking_mode = false;

    virtual void from_parameters(const Yolo8BodyPoseDetector *node);
    JS_OBJECT(JS_MEMBER(_time_unit),
              JS_MEMBER(step_interval),
              JS_MEMBER(enable_blocking_mode));
};
} // namespace redoxi_works::model_nodes::yolo8_body_pose_detector
