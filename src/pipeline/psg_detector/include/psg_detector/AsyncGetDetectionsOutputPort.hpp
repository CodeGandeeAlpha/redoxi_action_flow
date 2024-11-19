#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <psg_detector/GetDetectionsOutputSpec.hpp>

namespace redoxi_works
{
using AsyncGetDetectionsOutputPort = AsyncActionOutputPort<async_action_get_detections_output_port::PSGGetDetectionsOutputPortSpec>;

} // namespace redoxi_works
