#include <redoxi_samples_lib/random_image.hpp>

namespace redoxi_works
{
int random_image_with_shapes(cv::Mat &image, const cv::Size &size)
{
    //! Randomize the frame
    cv::RNG rng(cv::getTickCount());
    cv::Scalar randomColor(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    image = cv::Mat(size, CV_8UC3, randomColor);

    //! Add some random shapes
    int numShapes = rng.uniform(1, 5);
    for (int i = 0; i < numShapes; ++i) {
        int shapeType = rng.uniform(0, 3);
        cv::Point pt1(rng.uniform(0, image.cols), rng.uniform(0, image.rows));
        cv::Point pt2(rng.uniform(0, image.cols), rng.uniform(0, image.rows));
        cv::Scalar color(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));

        if (shapeType == 0) {
            cv::rectangle(image, pt1, pt2, color, -1);
        } else if (shapeType == 1) {
            cv::circle(image, pt1, rng.uniform(10, 50), color, -1);
        } else {
            std::vector<cv::Point> pts;
            for (int j = 0; j < 3; ++j) {
                pts.push_back(cv::Point(rng.uniform(0, image.cols), rng.uniform(0, image.rows)));
            }
            cv::fillPoly(image, std::vector<std::vector<cv::Point>>{pts}, color);
        }
    }
    return 0;
}

int random_image_with_text(cv::Mat &image, const cv::Size &size, const std::string &text,
                           const cv::Scalar &background_color, const cv::Scalar &text_color)
{
    //! Create the image with the specified background color
    image = cv::Mat(size, CV_8UC3, background_color);

    //! Set up text parameters
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;
    int baseline = 0;

    //! Calculate safe area for text
    int borderPadding = std::min(size.width, size.height) / 10;
    cv::Rect safeArea(borderPadding, borderPadding, size.width - 2 * borderPadding, size.height - 2 * borderPadding);

    //! Split text into lines
    std::vector<std::string> lines;
    std::istringstream iss(text);
    std::string line;
    while (std::getline(iss, line)) {
        lines.push_back(line);
    }

    //! Calculate text size and position
    std::vector<std::string> wrappedLines;
    for (const auto &l : lines) {
        std::string currentLine;
        std::istringstream wordStream(l);
        std::string word;
        while (wordStream >> word) {
            std::string testLine = currentLine + (currentLine.empty() ? "" : " ") + word;
            cv::Size textSize = cv::getTextSize(testLine, fontFace, fontScale, thickness, &baseline);
            if (textSize.width <= safeArea.width) {
                currentLine = testLine;
            } else {
                if (!currentLine.empty()) {
                    wrappedLines.push_back(currentLine);
                }
                currentLine = word;
            }
        }
        if (!currentLine.empty()) {
            wrappedLines.push_back(currentLine);
        }
    }

    //! Calculate total text height
    int totalTextHeight = wrappedLines.size() * (cv::getTextSize("Tg", fontFace, fontScale, thickness, &baseline).height + baseline);

    //! Draw text
    cv::Point textOrg(safeArea.x, safeArea.y + (safeArea.height - totalTextHeight) / 2);
    for (const auto &line : wrappedLines) {
        cv::putText(image, line, textOrg, fontFace, fontScale, text_color, thickness, cv::LINE_AA);
        textOrg.y += cv::getTextSize(line, fontFace, fontScale, thickness, &baseline).height + baseline;
    }

    return 0;
}
} // namespace redoxi_works
