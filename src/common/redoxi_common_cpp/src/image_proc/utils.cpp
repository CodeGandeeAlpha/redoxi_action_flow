#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <algorithm>
#include <numeric>

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

        // Determine color based on colorization mode
        switch (options.colorization_mode) {
            case DrawDetectionsOptions::ColorizationMode::ClassId:
                if (detection.category >= 0) {
                    box_color = cv::Scalar(detection.category * 10 % 256, 100, 100); // Example colorization by class id
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
