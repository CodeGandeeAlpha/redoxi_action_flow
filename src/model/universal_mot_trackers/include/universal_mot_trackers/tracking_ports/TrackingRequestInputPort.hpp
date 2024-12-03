#pragma once

#include <universal_mot_trackers/tracking_ports/TrackingRequestInputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>

namespace redoxi_works::model_nodes::tracking_ports::request_response
{
using TrackingRequestInputPort = AsyncActionInputPort<types::TrackingRequestInputPortSpec>;
} // namespace redoxi_works::model_nodes::tracking_ports::request_response