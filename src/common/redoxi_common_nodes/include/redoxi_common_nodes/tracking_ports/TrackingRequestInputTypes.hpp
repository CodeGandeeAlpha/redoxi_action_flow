#pragma once

#include <redoxi_common_nodes/visibility_control.h>
#include <redoxi_common_nodes/tracking_ports/TrackingRequestCommon.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>
namespace redoxi_works::tracking_ports::request_response::types
{

//! Input port types for tracking requests
using TrackingRequestSourceData = input_port_types::DefaultReceiveSourceData<TrackingRequestActionType>;

struct TrackingRequestInputPortSpec {
    using ActionType_t = TrackingRequestActionType;
    using ActionGoal_t = ActionType_t::Goal;
    using ActionDataTrait_t = TrackingRequestActionDataTrait;
    using TimeUnit_t = TimeUnit;
    using ReceiveSourceData_t = TrackingRequestSourceData;
    using InitConfig_t = input_port_types::DefaultInitConfig<TimeUnit_t, ActionType_t>;
};

static_assert(input_port_types::AsyncActionInputPortSpecConcept<TrackingRequestInputPortSpec>,
              "TrackingRequestInputPortSpec must satisfy AsyncActionInputPortSpecConcept");


} // namespace redoxi_works::tracking_ports::request_response::types