#pragma once

#include <redoxi_common_cpp/common_concepts.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_public_msgs/action/process_detections.hpp>

namespace redoxi_works::detection_ports::response_only::types
{
using TimeUnit = DefaultTimeUnit_t;
using DetectionResponseActionType = redoxi_public_msgs::action::ProcessDetections;
using DetectionResponseActionDataTrait = RedoxiActionDataTrait<DetectionResponseActionType>;
static_assert(RedoxiActionConcept<DetectionResponseActionType>, "DetectionResponseActionType must satisfy RedoxiActionConcept");
} // namespace redoxi_works::detection_ports::response_only::types