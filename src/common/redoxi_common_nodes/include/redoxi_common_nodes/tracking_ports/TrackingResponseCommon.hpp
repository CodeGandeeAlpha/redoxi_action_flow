#pragma once

#include <redoxi_common_nodes/visibility_control.h>
#include <redoxi_common_cpp/common_concepts.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_public_msgs/action/process_tracked_objects.hpp>
#include <redoxi_public_msgs/msg/process_tracked_objects_goal.hpp>

namespace redoxi_works::tracking_ports::response_only::types
{
using TimeUnit = DefaultTimeUnit_t;
using TrackingResponseActionType = redoxi_public_msgs::action::ProcessTrackedObjects;
using TrackingResponseActionDataTrait = RedoxiActionDataTrait<TrackingResponseActionType>;
using TrackingResponseGoalMsgType = redoxi_public_msgs::msg::ProcessTrackedObjectsGoal;
using TrackTargetType = redoxi_public_msgs::msg::TrackTarget;
static_assert(RedoxiActionConcept<TrackingResponseActionType>);

} // namespace redoxi_works::tracking_ports::response_only::types