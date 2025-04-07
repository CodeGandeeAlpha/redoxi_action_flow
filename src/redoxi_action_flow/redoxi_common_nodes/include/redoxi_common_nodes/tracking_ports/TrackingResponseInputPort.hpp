#pragma once

#include <redoxi_common_nodes/tracking_ports/TrackingResponseCommon.hpp>
#include <redoxi_common_nodes/tracking_ports/TrackingResponseInputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>

namespace redoxi_works::tracking_ports::response_only
{

using TrackingResponseInputPort = AsyncActionInputPort<types::TrackingResponseInputPortSpec>;

}