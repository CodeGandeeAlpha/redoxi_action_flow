#pragma once

#include <any>
#include <string>
#include <chrono>
#include <redoxi_common_cpp/visibility_control.h>
#include <opencv2/opencv.hpp>

namespace redoxi_works::data_types
{
struct ImageWithMetadata {
    cv::Mat image;
    std::string encoding; // follows ROS2 image encoding format
};
} // namespace redoxi_works::data_types