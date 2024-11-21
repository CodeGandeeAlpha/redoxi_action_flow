#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionActionOutputTypes.hpp>

namespace redoxi_works::detection_ports
{
using DetectionActionOutputPort = AsyncActionOutputPort<output_types::DetectionActionOutputPortSpec>;
} // namespace redoxi_works::detection_ports
