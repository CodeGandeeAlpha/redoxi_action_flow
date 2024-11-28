#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <opencv2/opencv.hpp>
#include <redoxi_public_msgs/msg/detection.hpp>

namespace redoxi_works::image_utils
{
/**
 * @brief Compute the size to resize to, keeping the aspect ratio, so that the resized image fits into the preferred size.
 * @param original_size The size of the original image.
 * @param preferred_size The preferred size of the resized image.
 * @return The size to resize to.
 */
cv::Size compute_resize_to_fit_and_keep_aspect_ratio(const cv::Size &original_size, const cv::Size &preferred_size);

/**
 * @brief Options for drawing detections on an image.
 */
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

/**
 * @brief Draw detections on an image.
 */
void draw_detections(cv::Mat *output,
                     const std::vector<redoxi_public_msgs::msg::Detection> &detections,
                     const DrawDetectionsOptions &options = DrawDetectionsOptions());
} // namespace redoxi_works::image_utils
