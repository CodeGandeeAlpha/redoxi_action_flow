#pragma once

#include <redoxi_common_cpp/common_concepts.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/visibility_control.h>
#include <redoxi_public_msgs/action/process_detections_by_frame.hpp>

namespace redoxi_works::detection_ports::request_response::types
{
using TimeUnit = DefaultTimeUnit_t;
using DetectionRequestActionType = redoxi_public_msgs::action::ProcessDetectionsByFrame;
using DetectionRequestActionDataTrait = RedoxiActionDataTrait<DetectionRequestActionType>;
static_assert(RedoxiActionConcept<DetectionRequestActionType>, "DetectionRequestActionType must satisfy RedoxiActionConcept");
} // namespace redoxi_works::detection_ports::request_response::types