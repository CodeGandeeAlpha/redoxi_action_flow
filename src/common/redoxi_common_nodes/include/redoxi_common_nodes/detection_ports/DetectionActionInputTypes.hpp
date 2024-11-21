#pragma once

#include <any>
#include <redoxi_public_msgs/action/process_detections_by_frame.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>

namespace redoxi_works::detection_ports::input_types
{

using TimeUnit = DefaultTimeUnit_t;
using DetectionActionType = redoxi_public_msgs::action::ProcessDetectionsByFrame;
using DetectionActionDataTrait = RedoxiActionDataTrait<DetectionActionType>;
static_assert(RedoxiActionConcept<DetectionActionType>, "DetectionActionType must satisfy RedoxiActionConcept");

using DetectionSourceData = input_port_types::DefaultReceiveSourceData<DetectionActionType>;
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

} // namespace redoxi_works::detection_ports::input_types