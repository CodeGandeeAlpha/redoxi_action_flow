#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <psg_tracker/GetTrackTargetsOutputSpec.hpp>

namespace redoxi_works
{
using AsyncGetTrackTargetsOutputPort = AsyncActionOutputPort<async_action_get_track_targets_output_port::PSGGetTrackTargetsOutputPortSpec>;

} // namespace redoxi_works
