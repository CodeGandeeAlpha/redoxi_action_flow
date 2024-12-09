#pragma once

#include <universal_mot_trackers/tracking_ports/TrackingRequestCommon.hpp>
#include <universal_mot_trackers/tracking_ports/TrackingRequestOutputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>

namespace redoxi_works::model_nodes::tracking_ports::request_response
{

using TrackingRequestOutputPort = AsyncActionOutputPort<types::TrackingRequestOutputPortSpec>;

}