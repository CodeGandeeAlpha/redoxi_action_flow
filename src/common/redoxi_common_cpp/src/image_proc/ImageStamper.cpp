#include <redoxi_common_cpp/_pch.hpp>
#include <redoxi_common_cpp/image_proc/ImageStamper.hpp>

namespace redoxi_works
{
ImageStamper::ImageStamper(const cv::Mat &image)
    : m_source_image(image)
{
}

ImageStamper &ImageStamper::clear()
{
    m_text_infos.clear();
    return *this;
}

ImageStamper &ImageStamper::add_text(const std::string &text,
                                     double scale,
                                     std::optional<cv::Scalar> text_color,
                                     std::optional<cv::Scalar> background_color)
{
    cv::Scalar final_text_color = text_color.value_or(DEFAULT_TEXT_COLOR);
    cv::Scalar final_background_color;

    if (background_color.has_value()) {
        final_background_color = background_color.value();
    } else {
        // Invert the text color for the background
        final_background_color = cv::Scalar(255, 255, 255) - final_text_color;
    }

    TextInfo info{
        text,
        final_text_color,
        final_background_color,
        scale};

    m_text_infos.push_back(info);

    return *this;
}

cv::Mat ImageStamper::stamp(bool in_place)
{
    //! Create the final image, either modify the source image in place or return a copy
    cv::Mat final_image;
    if (!in_place) {
        final_image = m_source_image.clone();
    } else {
        final_image = m_source_image;
    }

    //! If there's no text to stamp, return the image as is
    if (m_text_infos.empty()) {
        return final_image;
    }

    //! Set up text rendering parameters
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.8; // Base font size
    int thickness = 1;
    int lineSpacing = 5;
    int padding = 10;
    int maxWidth = final_image.cols - 2 * padding;

    std::vector<std::vector<std::string>> textBlocks;
    std::vector<cv::Scalar> backgroundColors;
    std::vector<cv::Scalar> textColors;
    std::vector<double> scales;
    int totalHeight = padding;

    //! Process each text info and break it into lines
    for (const auto &info : m_text_infos) {
        std::vector<std::string> textLines;
        std::string currentLine;
        std::istringstream iss(info.text);
        std::string word;

        //! Break text into lines that fit within maxWidth
        while (iss >> word) {
            std::string testLine = currentLine + (currentLine.empty() ? "" : " ") + word;
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(testLine, fontFace, info.scale * fontScale, thickness, &baseline);

            if (textSize.width > maxWidth) {
                if (!currentLine.empty()) {
                    textLines.push_back(currentLine);
                    currentLine = word;
                } else {
                    textLines.push_back(word);
                }
            } else {
                currentLine = testLine;
            }
        }

        if (!currentLine.empty()) {
            textLines.push_back(currentLine);
        }

        textBlocks.push_back(textLines);
        backgroundColors.push_back(info.background_color);
        textColors.push_back(info.text_color);
        scales.push_back(info.scale);

        totalHeight += textLines.size() * (lineSpacing + cv::getTextSize("Tg", fontFace, info.scale * fontScale, thickness, nullptr).height);
        totalHeight += padding; // Add padding between text blocks
    }

    totalHeight += padding;

    //! Render each text block
    int y = padding;
    for (size_t i = 0; i < textBlocks.size(); ++i) {
        const auto &lines = textBlocks[i];
        const auto &bgColor = backgroundColors[i];
        const auto &textColor = textColors[i];
        const auto &scale = scales[i];

        int blockHeight = lines.size() * (lineSpacing + cv::getTextSize("Tg", fontFace, scale * fontScale, thickness, nullptr).height);

        //! Create a background for the text block
        cv::Rect backgroundRect(0, y - padding / 2, final_image.cols, blockHeight + padding);
        cv::rectangle(final_image, backgroundRect, bgColor, cv::FILLED);

        for (const auto &line : lines) {
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(line, fontFace, scale * fontScale, thickness, &baseline);

            cv::Point textOrg(padding, y + textSize.height);
            cv::putText(final_image, line, textOrg, fontFace, scale * fontScale, textColor, thickness, cv::LINE_AA);

            y += lineSpacing + textSize.height;
        }

        y += padding; // Add padding after each text block
    }

    return final_image;
}
} // namespace redoxi_works
