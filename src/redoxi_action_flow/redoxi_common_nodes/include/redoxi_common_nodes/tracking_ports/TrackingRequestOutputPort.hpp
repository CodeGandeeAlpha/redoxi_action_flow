#pragma once

#include <redoxi_common_nodes/tracking_ports/TrackingRequestCommon.hpp>
#include <redoxi_common_nodes/tracking_ports/TrackingRequestOutputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>

namespace redoxi_works::tracking_ports::request_response
{

using TrackingRequestOutputPort = AsyncActionOutputPort<types::TrackingRequestOutputPortSpec>;

}