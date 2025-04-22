#pragma once

#include <redoxi_common_nodes/visibility_control.h>
#include <redoxi_common_nodes/detection_ports/DetectionRequestOutputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>

namespace redoxi_works::detection_ports::request_response
{
using DetectionRequestOutputPort = AsyncActionOutputPort<types::DetectionRequestOutputPortSpec>;
}