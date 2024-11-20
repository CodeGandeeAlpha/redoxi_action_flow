#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
#include <psg_pose_detector/GetKeypointsOutputSpec.hpp>

namespace redoxi_works
{
using AsyncGetKeypointsOutputPort = AsyncActionOutputPort<async_action_get_keypoints_output_port::PSGGetKeypointsOutputPortSpec>;

} // namespace redoxi_works
