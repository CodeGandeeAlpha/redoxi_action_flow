#pragma once

#include <redoxi_samples_lib/redoxi_samples_lib.hpp>
#include <opencv2/opencv.hpp>
namespace redoxi_works
{

/**
 * @brief Generates a random image with shapes.
 *
 * This function creates a random image with a background color and adds
 * random shapes (rectangles, circles, or triangles) to it.
 *
 * @param[out] image The output cv::Mat where the generated image will be stored.
 * @param[in] size The size of the image to be generated.
 * @return Returns 0 on success.
 */
int random_image_with_shapes(cv::Mat &image, const cv::Size &size);

/**
 * @brief Generates an image with text.
 *
 * This function creates an image with a specified background color and adds
 * the given text to it. The text is automatically wrapped to fit within the image.
 *
 * @param[out] image The output cv::Mat where the generated image will be stored.
 * @param[in] size The size of the image to be generated.
 * @param[in] text The text to be added to the image.
 * @param[in] background_color The background color of the image (default is black).
 * @param[in] text_color The color of the text (default is white).
 * @return Returns 0 on success.
 */
int random_image_with_text(cv::Mat &image, const cv::Size &size,
                           const std::string &text,
                           const cv::Scalar &background_color = cv::Scalar(),
                           const cv::Scalar &text_color = cv::Scalar());

} // namespace redoxi_works