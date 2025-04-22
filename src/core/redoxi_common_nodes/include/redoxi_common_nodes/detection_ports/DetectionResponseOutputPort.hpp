#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionResponseOutputTypes.hpp>

namespace redoxi_works::detection_ports::response_only
{
using DetectionResponseOutputPort = AsyncActionOutputPort<types::DetectionResponseOutputPortSpec>;
} // namespace redoxi_works::detection_ports::response_only
