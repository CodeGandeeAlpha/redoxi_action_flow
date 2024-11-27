#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>
#include <psg_tracker/GetTrackTargetsInputSpec.hpp>

namespace redoxi_works
{
using AsyncGetTrackTargetsInputPort = AsyncActionInputPort<async_action_get_track_targets_input_port::PSGGetTrackTargetsInputPortSpec>;

} // namespace redoxi_works
