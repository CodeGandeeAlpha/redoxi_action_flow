#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <opencv2/opencv.hpp>

namespace redoxi_works::image_utils
{
/**
 * @brief Compute the size to resize to, keeping the aspect ratio, so that the resized image fits into the preferred size.
 * @param original_size The size of the original image.
 * @param preferred_size The preferred size of the resized image.
 * @return The size to resize to.
 */
cv::Size compute_resize_to_fit_and_keep_aspect_ratio(const cv::Size &original_size, const cv::Size &preferred_size);
} // namespace redoxi_works::image_utils
