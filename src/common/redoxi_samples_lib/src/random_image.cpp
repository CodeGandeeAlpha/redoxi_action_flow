#include <redoxi_samples_lib/random_image.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>

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

int random_image_with_text(cv::Mat &image, const cv::Size &size, std::optional<std::string> text,
                           std::optional<cv::Scalar> text_color, std::optional<cv::Scalar> background_color)
{
    //! Create the image with the specified background color or a random color if not provided
    cv::RNG rng(cv::getTickCount());
    cv::Scalar actualBackgroundColor = background_color.value_or(
        cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256)));
    image = cv::Mat(size, CV_8UC3, actualBackgroundColor);

    //! Set up text parameters
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1.0;
    int thickness = 2;
    int baseline = 0;

    //! Calculate safe area for text
    int borderPadding = std::min(size.width, size.height) / 10;
    cv::Rect safeArea(borderPadding, borderPadding, size.width - 2 * borderPadding, size.height - 2 * borderPadding);

    //! Generate text if not provided
    std::vector<std::string> lines;
    if (!text.has_value()) {
        //! Generate UUID
        boost::uuids::random_generator gen;
        boost::uuids::uuid uuid = gen();
        lines.push_back(boost::uuids::to_string(uuid));

        //! Generate current time
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << ":" << std::setfill('0') << std::setw(3) << now_ms.count();
        lines.push_back(ss.str());
    } else {
        //! Split provided text into lines
        std::istringstream iss(text.value());
        std::string line;
        while (std::getline(iss, line)) {
            lines.push_back(line);
        }
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

    //! Set text color or use inverted background color if not provided
    cv::Scalar actualTextColor;
    if (text_color.has_value()) {
        actualTextColor = text_color.value();
    } else {
        actualTextColor = cv::Scalar(255, 255, 255) - actualBackgroundColor;
    }

    //! Draw text
    cv::Point textOrg(safeArea.x, safeArea.y + (safeArea.height - totalTextHeight) / 2);
    for (const auto &line : wrappedLines) {
        cv::putText(image, line, textOrg, fontFace, fontScale, actualTextColor, thickness, cv::LINE_AA);
        textOrg.y += cv::getTextSize(line, fontFace, fontScale, thickness, &baseline).height + baseline;
    }

    return 0;
}
} // namespace redoxi_works
