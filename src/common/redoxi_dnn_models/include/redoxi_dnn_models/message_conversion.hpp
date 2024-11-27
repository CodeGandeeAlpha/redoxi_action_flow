#pragma once

#include <redoxi_dnn_models/visibility_control.h>
#include <redoxi_dnn_models/detection_types.hpp>
#include <redoxi_public_msgs/msg/detection.hpp>
#include <redoxi_public_msgs/msg/keypoints.hpp>

#include <vector>

namespace redoxi_works::inference::conversion
{
namespace det_types = redoxi_works::inference::detection::types;

void to_ros_msg(redoxi_public_msgs::msg::Detection *output_msg,
                const det_types::DetectedObject &object);

void to_ros_msg(std::vector<redoxi_public_msgs::msg::Detection> *output_msg,
                const det_types::SingleImageOutput &image_detections);

void to_ros_msg(redoxi_public_msgs::msg::Keypoints *output_msg,
                const std::vector<det_types::Keypoint> &keypoints);

void from_ros_msg(det_types::SingleImageOutput *output_detections,
                  const std::vector<redoxi_public_msgs::msg::Detection> &input_msg);

void from_ros_msg(det_types::DetectedObject *output_object,
                  const redoxi_public_msgs::msg::Detection &input_msg);

void from_ros_msg(std::vector<det_types::Keypoint> *output_keypoints,
                  const redoxi_public_msgs::msg::Keypoints &input_msg);

std::vector<det_types::Keypoint> from_ros_msg(const redoxi_public_msgs::msg::Keypoints &input_msg);
det_types::SingleImageOutput from_ros_msg(const std::vector<redoxi_public_msgs::msg::Detection> &input_msg);
det_types::DetectedObject from_ros_msg(const redoxi_public_msgs::msg::Detection &input_msg);

} // namespace redoxi_works::inference::conversion
