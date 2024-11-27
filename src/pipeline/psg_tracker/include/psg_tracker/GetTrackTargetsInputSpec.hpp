#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>
#include <psg_private_msgs/action/process_track_targets_by_persons.hpp>

namespace redoxi_works
{

namespace async_action_get_track_targets_input_port
{
//! The specification for the track targets input port
using TimeUnit = DefaultTimeUnit_t;
using TrackTargetsActionType = psg_private_msgs::action::ProcessTrackTargetsByPersons;
using TrackTargetsActionDataTrait = RedoxiActionDataTrait<TrackTargetsActionType>;

using PSGGetTrackTargetsInputPortSpec =
    input_port_types::DefaultAsyncActionInputPortSpec<TrackTargetsActionType, TrackTargetsActionDataTrait, TimeUnit>;

} // namespace async_action_get_track_targets_input_port

} // namespace redoxi_works
