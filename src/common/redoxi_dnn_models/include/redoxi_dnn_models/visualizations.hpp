#pragma once

#include <redoxi_dnn_models/visibility_control.h>
#include <redoxi_dnn_models/detection_types.hpp>
#include <opencv2/opencv.hpp>

namespace redoxi_works::dnn_models::visualizations
{
struct DrawDetectionsOptions {
    enum class ColorizationMode {
        None,    //!< if color is defined in the detection object, use it, otherwise use the default color
        ClassId, //!< Colorize by class id
        Random   //!< Random colorization
    };

    ColorizationMode colorization_mode = ColorizationMode::None;
    cv::Scalar default_box_color = cv::Scalar(0, 0, 255);
    bool draw_bboxes = true;
    int box_thickness = 2;
    bool draw_keypoints = true;
    int keypoint_radius = 2;
};

void draw_detections(cv::Mat *output,
                     const inference::detection::types::SingleImageOutput &detections,
                     const DrawDetectionsOptions &options = DrawDetectionsOptions());
} // namespace redoxi_works::dnn_models::visualizations
