#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <algorithm>
#include <numeric>

static const std::vector<cv::Scalar> color_palette_rgb = {
    cv::Scalar(255, 10, 0),    // red
    cv::Scalar(47, 79, 79),    // darkslategray
    cv::Scalar(85, 107, 47),   // darkolivegreen
    cv::Scalar(139, 69, 19),   // saddlebrown
    cv::Scalar(107, 142, 35),  // olivedrab
    cv::Scalar(25, 25, 112),   // midnightblue
    cv::Scalar(112, 128, 144), // slategray
    cv::Scalar(139, 0, 0),     // darkred
    cv::Scalar(0, 128, 0),     // green
    cv::Scalar(60, 179, 113),  // mediumseagreen
    cv::Scalar(188, 143, 143), // rosybrown
    cv::Scalar(184, 134, 11),  // darkgoldenrod
    cv::Scalar(189, 183, 107), // darkkhaki
    cv::Scalar(0, 139, 139),   // darkcyan
    cv::Scalar(70, 130, 180),  // steelblue
    cv::Scalar(0, 0, 128),     // navy
    cv::Scalar(210, 105, 30),  // chocolate
    cv::Scalar(154, 205, 50),  // yellowgreen
    cv::Scalar(50, 205, 50),   // limegreen
    cv::Scalar(143, 188, 143), // darkseagreen
    cv::Scalar(139, 0, 139),   // darkmagenta
    cv::Scalar(72, 209, 204),  // mediumturquoise
    cv::Scalar(153, 50, 204),  // darkorchid
    cv::Scalar(255, 69, 0),    // orangered
    cv::Scalar(255, 140, 0),   // darkorange
    cv::Scalar(255, 215, 0),   // gold
    cv::Scalar(106, 90, 205),  // slateblue
    cv::Scalar(0, 0, 205),     // mediumblue
    cv::Scalar(0, 255, 0),     // lime
    cv::Scalar(0, 255, 127),   // springgreen
    cv::Scalar(220, 20, 60),   // crimson
    cv::Scalar(0, 191, 255),   // deepskyblue
    cv::Scalar(244, 164, 96),  // sandybrown
    cv::Scalar(0, 0, 255),     // blue
    cv::Scalar(160, 32, 240),  // purple3
    cv::Scalar(173, 255, 47),  // greenyellow
    cv::Scalar(255, 99, 71),   // tomato
    cv::Scalar(218, 112, 214), // orchid
    cv::Scalar(176, 196, 222), // lightsteelblue
    cv::Scalar(255, 0, 255),   // fuchsia
    cv::Scalar(30, 144, 255),  // dodgerblue
    cv::Scalar(219, 112, 147), // palevioletred
    cv::Scalar(250, 128, 114), // salmon
    cv::Scalar(255, 255, 84),  // laserlemon
    cv::Scalar(221, 160, 221), // plum
    cv::Scalar(255, 20, 147),  // deeppink
    cv::Scalar(245, 222, 179), // wheat
    cv::Scalar(175, 238, 238), // paleturquoise
    cv::Scalar(152, 251, 152), // palegreen
    cv::Scalar(127, 255, 212), // aquamarine
    cv::Scalar(255, 105, 180)  // hotpink
};

namespace redoxi_works::image_utils
{
cv::Size compute_resize_to_fit_and_keep_aspect_ratio(const cv::Size &original_size, const cv::Size &preferred_size)
{
    auto resize_ratio = std::min(preferred_size.width / (float)original_size.width,
                                 preferred_size.height / (float)original_size.height);

    // no clamping, using rounding
    cv::Size output(
        static_cast<int>(std::round(original_size.width * resize_ratio)),
        static_cast<int>(std::round(original_size.height * resize_ratio)));

    // no 0 size
    output.width = std::max(1, output.width);
    output.height = std::max(1, output.height);

    return output;
}

void draw_detections(cv::Mat *output,
                     const std::vector<redoxi_public_msgs::msg::Detection> &detections,
                     const DrawDetectionsOptions &options)
{
    if (!output) {
        return;
    }

    if (detections.empty()) {
        return;
    }

    // if output is empty image, get size from first detection
    if (output->empty()) {
        cv::Size size(detections[0].frame_metadata.width, detections[0].frame_metadata.height);
        if (size.width <= 0 || size.height <= 0) {
            // empty image and unknown size, cannot draw
            return;
        }
        *output = cv::Mat::zeros(size, CV_8UC3);
    }

    for (const auto &detection : detections) {
        cv::Scalar box_color = options.default_box_color;

        // RDX_INFO_DEV(nullptr, __func__, false, "detection category: {}", detection.category);

        // Determine color based on colorization mode
        switch (options.colorization_mode) {
            case DrawDetectionsOptions::ColorizationMode::ClassId:
                if (detection.category >= 0) {
                    box_color = color_palette_rgb[detection.category % color_palette_rgb.size()];
                    //! Convert box_color from RGB to BGR
                    std::swap(box_color[0], box_color[2]);
                }
                break;
            case DrawDetectionsOptions::ColorizationMode::Random:
                box_color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256); // Random color
                break;
            default:
                break;
        }

        // Draw bounding box if enabled
        if (options.draw_bboxes) {
            cv::Rect bbox(detection.bbox.x, detection.bbox.y,
                          detection.bbox.width, detection.bbox.height);
            cv::rectangle(*output, bbox, box_color, options.box_thickness);
        }

        // Draw keypoints if enabled
        if (options.draw_keypoints) {
            const auto &keypoints = detection.keypoints.keypoints_2;
            if (keypoints.empty()) {
                // no keypoints, skip
                continue;
            }
            for (const auto &kp : keypoints) {
                cv::Point2f pt(kp.x, kp.y);
                cv::circle(*output, pt, options.keypoint_radius, box_color, -1);
            }

            const auto &keypoint_edges = detection.keypoints.keypoint_edges;
            auto num_edges = keypoint_edges.size() / 2;
            for (size_t i = 0; i < num_edges; ++i) {
                auto u = keypoint_edges[2 * i];
                auto v = keypoint_edges[2 * i + 1];
                auto p_u = keypoints[u];
                auto p_v = keypoints[v];
                cv::line(*output, cv::Point(p_u.x, p_u.y), cv::Point(p_v.x, p_v.y), box_color, 2);
            }
        }
    }
}
} // namespace redoxi_works::image_utils
