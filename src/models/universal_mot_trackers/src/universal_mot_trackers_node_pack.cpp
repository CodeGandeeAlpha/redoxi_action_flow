#include <universal_mot_trackers/UniversalMotTrackerNode.hpp>
#include <universal_mot_trackers/TrackerDriverNode.hpp>

namespace redoxi_works::node_pack::tracking
{
using UniversalMotTracker = model_nodes::universal_mot_trackers::UniversalMotTrackerNode;
using MotTrackerDriver = model_nodes::universal_mot_trackers::TrackerDriverNode;
} // namespace redoxi_works::node_pack::tracking

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::node_pack::tracking::UniversalMotTracker)
RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::node_pack::tracking::MotTrackerDriver)
