#pragma once

#include <redoxi_common_nodes/detection_ports/DetectionRequestInputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>

namespace redoxi_works::detection_ports::request_response
{
using DetectionRequestInputPort = AsyncActionInputPort<types::DetectionRequestInputPortSpec>;
}
