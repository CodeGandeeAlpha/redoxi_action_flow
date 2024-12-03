#pragma once

#include <universal_mot_trackers/visibility_control.h>
#include <redoxi_common_cpp/common_concepts.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_public_msgs/action/process_track_by_detection.hpp>

namespace redoxi_works::model_nodes::tracking_ports::request_response::types
{
using TimeUnit = DefaultTimeUnit_t;
using TrackingRequestActionType = redoxi_public_msgs::action::ProcessTrackByDetection;
using TrackingRequestActionDataTrait = RedoxiActionDataTrait<TrackingRequestActionType>;
static_assert(RedoxiActionConcept<TrackingRequestActionType>);

} // namespace redoxi_works::model_nodes::tracking_ports::request_response::types