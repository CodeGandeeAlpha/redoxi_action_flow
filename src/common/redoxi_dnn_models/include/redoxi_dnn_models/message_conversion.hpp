#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
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
} // namespace redoxi_works::inference::conversion
