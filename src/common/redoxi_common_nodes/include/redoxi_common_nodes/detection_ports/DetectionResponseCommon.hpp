#pragma once

#include <redoxi_common_cpp/common_concepts.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_public_msgs/action/process_detections.hpp>
#include <redoxi_public_msgs/msg/process_detections_goal.hpp>

namespace redoxi_works::detection_ports::response_only::types
{
using TimeUnit = DefaultTimeUnit_t;
using DetectionResponseActionType = redoxi_public_msgs::action::ProcessDetections;
using DetectionResponseActionDataTrait = RedoxiActionDataTrait<DetectionResponseActionType>;
using DetectionResponseGoalMsgType = redoxi_public_msgs::msg::ProcessDetectionsGoal;
static_assert(RedoxiActionConcept<DetectionResponseActionType>, "DetectionResponseActionType must satisfy RedoxiActionConcept");
} // namespace redoxi_works::detection_ports::response_only::types