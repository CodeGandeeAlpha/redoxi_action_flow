#include <redoxi_common_nodes/_pch.hpp>
#include <redoxi_common_nodes/driver_nodes/DetectionDriver.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace redoxi_works::node_pack::detection
{
using DetectionDriver = redoxi_works::common_nodes::drivers::DetectionDriver;
}

RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::node_pack::detection::DetectionDriver)
