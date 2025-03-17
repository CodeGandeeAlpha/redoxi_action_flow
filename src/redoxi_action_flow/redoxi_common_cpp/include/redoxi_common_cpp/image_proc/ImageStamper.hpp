#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

namespace redoxi_works
{

/**
 * @brief A class to stamp an image with text in a block on the top-left corner.
 */
class ImageStamper
{
  public:
    ImageStamper(const cv::Mat &image);
    virtual ~ImageStamper() = default;

    inline static const cv::Scalar DEFAULT_TEXT_COLOR = cv::Scalar(50, 50, 50);

    /**
     * @brief Add a text to the image.
     * @param text The text to add.
     * @param scale The scale of the text, default is 1.0.
     * @param text_color The color of the text, if not provided, it will be the default text color.
     * @param background_color The color of the background, if not provided, it will be the inverted color of text_color.
     */
    virtual ImageStamper &add_text(const std::string &text,
                                   double scale = 1.0,
                                   std::optional<cv::Scalar> text_color = std::nullopt,
                                   std::optional<cv::Scalar> background_color = std::nullopt);

    virtual ImageStamper &clear();

    /**
     * @brief Stamp the image with the added texts.
     * @param in_place Whether to modify the source image in place. By default, it will return a copy.
     * @return The stamped image.
     */
    virtual cv::Mat stamp(bool in_place = false);

    const cv::Mat &get_source_image() const
    {
        return m_source_image;
    }

  protected:
    struct TextInfo {
        std::string text;
        cv::Scalar text_color;
        cv::Scalar background_color;
        double scale;
    };

    cv::Mat m_source_image;
    std::vector<TextInfo> m_text_infos;
};

} // namespace redoxi_works