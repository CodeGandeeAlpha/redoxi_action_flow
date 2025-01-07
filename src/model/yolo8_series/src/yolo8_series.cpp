#include <yolo8_series/_pch.hpp>

#include <yolo8_series/yolo8_series.hpp>
#include <yolo8_series/bodypose/Yolo8BodyPoseNode.hpp>
#include <yolo8_series/detection/Yolo8ObjectDetNode.hpp>

namespace redoxi_works::model_nodes::yolo8
{
} // namespace redoxi_works::model_nodes::yolo8

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::model_nodes::yolo8::Yolo8BodyPoseNode)
RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::model_nodes::yolo8::Yolo8ObjectDetNode)