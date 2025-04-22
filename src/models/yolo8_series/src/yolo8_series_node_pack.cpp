#include <yolo8_series/detection/Yolo8ObjectDetNode.hpp>
#include <yolo8_series/bodypose/Yolo8BodyPoseNode.hpp>

namespace redoxi_works::node_pack::detection::yolo8
{
using ObjectDetector = model_nodes::yolo8::Yolo8ObjectDetNode;
using BodyPoseDetector = model_nodes::yolo8::Yolo8BodyPoseNode;
} // namespace redoxi_works::node_pack::detection::yolo8

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::node_pack::detection::yolo8::ObjectDetector)
RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::node_pack::detection::yolo8::BodyPoseDetector)
