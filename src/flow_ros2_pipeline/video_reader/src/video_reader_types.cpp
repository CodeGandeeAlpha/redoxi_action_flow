#include <video_reader/video_reader.hpp>
#include <video_reader/video_reader_types.hpp>

namespace FlowRos2Pipeline
{
void OpencvVideoReaderInitConfig::from_parameters(OpencvVideoReader *node)
{
    auto logger_ = node->get_logger();

    this->source_file = node->get_parameter("source_file").as_string();
    this->source_camera_index = node->get_parameter("source_camera_index").as_int();
    this->start_frame_number = node->get_parameter("start_frame_number").as_int();
    this->end_frame_number = node->get_parameter("end_frame_number").as_int();

    RCLCPP_INFO(logger_, "source_file: %s", this->source_file.c_str());
    RCLCPP_INFO(logger_, "source_camera_index: %d", this->source_camera_index);
    RCLCPP_INFO(logger_, "start_frame_number: %d", this->start_frame_number);
    RCLCPP_INFO(logger_, "end_frame_number: %d", this->end_frame_number);
}

void OpencvVideoReaderRuntimeConfig::from_parameters(OpencvVideoReader *node)
{
    auto logger_ = node->get_logger();
    this->frame_interval_ms = node->get_parameter("frame_interval_ms").as_double();
    RCLCPP_INFO(logger_, "frame_interval_ms: %f", this->frame_interval_ms);

    this->image_width = node->get_parameter("image_width").as_int();
    RCLCPP_INFO(logger_, "image_width: %d", this->image_width);
    this->image_height = node->get_parameter("image_height").as_int();
    RCLCPP_INFO(logger_, "image_height: %d", this->image_height);
}
} // namespace FlowRos2Pipeline