#pragma once

#include <redoxi_common_nodes/tracking_ports/TrackingRequestInputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>

namespace redoxi_works::tracking_ports::request_response
{
using TrackingRequestInputPort = AsyncActionInputPort<types::TrackingRequestInputPortSpec>;
} // namespace redoxi_works::tracking_ports::request_response