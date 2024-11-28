#pragma once

#include <redoxi_common_nodes/detection_ports/DetectionRequestCommon.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>

namespace redoxi_works::detection_ports::request_response::types
{

using DetectionRequestSourceData = input_port_types::DefaultReceiveSourceData<DetectionRequestActionType>;
struct DetectionRequestInputPortSpec {
    using ActionType_t = DetectionRequestActionType;
    using ActionGoal_t = ActionType_t::Goal;
    using ActionDataTrait_t = DetectionRequestActionDataTrait;
    using TimeUnit_t = TimeUnit;
    using ReceiveSourceData_t = DetectionRequestSourceData;
    using InitConfig_t = input_port_types::DefaultInitConfig<TimeUnit>;
};
static_assert(input_port_types::AsyncActionInputPortSpecConcept<DetectionRequestInputPortSpec>,
              "DetectionRequestInputPortSpec must satisfy AsyncActionInputPortSpecConcept");

} // namespace redoxi_works::detection_ports::request_response::types