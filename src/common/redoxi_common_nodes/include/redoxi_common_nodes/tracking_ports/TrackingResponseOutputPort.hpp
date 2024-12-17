#pragma once

#include <redoxi_common_nodes/tracking_ports/TrackingResponseCommon.hpp>
#include <redoxi_common_nodes/tracking_ports/TrackingResponseOutputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>

namespace redoxi_works::tracking_ports::response_only
{

using TrackingResponseOutputPort = AsyncActionOutputPort<types::TrackingResponseOutputPortSpec>;

}