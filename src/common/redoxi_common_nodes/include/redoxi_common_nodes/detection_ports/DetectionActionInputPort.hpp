#pragma once

#include <any>
#include <redoxi_common_nodes/detection_ports/DetectionActionInputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>

namespace redoxi_works::detection_ports
{
using DetectionActionInputPort = AsyncActionInputPort<input_types::DetectionActionInputPortSpec>;
}
