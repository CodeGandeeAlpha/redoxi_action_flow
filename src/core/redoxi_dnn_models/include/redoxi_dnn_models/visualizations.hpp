#pragma once

#include <redoxi_dnn_models/visibility_control.h>
#include <redoxi_dnn_models/detection_types.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <opencv2/opencv.hpp>

namespace redoxi_works::dnn_models::visualizations
{
using DrawDetectionsOptions = redoxi_works::image_utils::DrawDetectionsOptions;

void draw_detections(cv::Mat *output,
                     const inference::detection::types::SingleImageOutput &detections,
                     const DrawDetectionsOptions &options = DrawDetectionsOptions());
} // namespace redoxi_works::dnn_models::visualizations
