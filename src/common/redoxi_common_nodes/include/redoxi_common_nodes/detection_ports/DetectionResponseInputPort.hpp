#pragma once

#include <redoxi_common_nodes/detection_ports/DetectionResponseInputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>

namespace redoxi_works::detection_ports::response_only
{
using DetectionResponseInputPort = AsyncActionInputPort<types::DetectionResponseInputPortSpec>;
}