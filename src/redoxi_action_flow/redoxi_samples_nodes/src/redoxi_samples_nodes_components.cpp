#include <redoxi_samples_nodes/sinks/FrameRelayNode.hpp>
#include <redoxi_samples_nodes/sinks/DetectionRelayNode.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace redoxi_works::node_pack::samples
{
using FrameRelayNode = redoxi_works::samples::FrameRelayNode;
using DetectionRelayNode = redoxi_works::samples::DetectionRelayNode;
} // namespace redoxi_works::node_pack::samples

RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::node_pack::samples::FrameRelayNode)
RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::node_pack::samples::DetectionRelayNode)