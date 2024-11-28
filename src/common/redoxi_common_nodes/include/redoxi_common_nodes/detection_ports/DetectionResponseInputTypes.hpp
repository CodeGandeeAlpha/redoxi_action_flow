#pragma once

#include <redoxi_common_nodes/detection_ports/DetectionResponseCommon.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>

namespace redoxi_works::detection_ports::response_only::types
{

using ReceiveSourceData = input_port_types::DefaultReceiveSourceData<DetectionResponseActionType>;
struct DetectionResponseInputPortSpec {
    using ActionType_t = DetectionResponseActionType;
    using ActionGoal_t = ActionType_t::Goal;
    using ActionDataTrait_t = DetectionResponseActionDataTrait;
    using TimeUnit_t = TimeUnit;
    using ReceiveSourceData_t = ReceiveSourceData;
    using InitConfig_t = input_port_types::DefaultInitConfig<TimeUnit>;
};
static_assert(input_port_types::AsyncActionInputPortSpecConcept<DetectionResponseInputPortSpec>,
              "DetectionResponseInputPortSpec must satisfy AsyncActionInputPortSpecConcept");
} // namespace redoxi_works::detection_ports::response_only::types