#pragma once

#include <redoxi_common_nodes/visibility_control.h>
#include <redoxi_common_nodes/tracking_ports/TrackingResponseCommon.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>
namespace redoxi_works::tracking_ports::response_only::types
{

//! Input port types for tracking requests
using TrackingResponseSourceData = input_port_types::DefaultReceiveSourceData<TrackingResponseActionType>;

struct TrackingResponseInputPortSpec {
    using ActionType_t = TrackingResponseActionType;
    using ActionGoal_t = ActionType_t::Goal;
    using ActionDataTrait_t = TrackingResponseActionDataTrait;
    using TimeUnit_t = TimeUnit;
    using ReceiveSourceData_t = TrackingResponseSourceData;
    using InitConfig_t = input_port_types::DefaultInitConfig<TimeUnit_t, ActionType_t>;
};

static_assert(input_port_types::AsyncActionInputPortSpecConcept<TrackingResponseInputPortSpec>,
              "TrackingResponseInputPortSpec must satisfy AsyncActionInputPortSpecConcept");


} // namespace redoxi_works::tracking_ports::response_only::types