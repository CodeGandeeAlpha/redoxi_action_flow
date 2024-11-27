#include <redoxi_dnn_models/visualizations.hpp>

namespace redoxi_works::dnn_models::visualizations
{
void draw_detections(cv::Mat *output,
                     const inference::detection::types::SingleImageOutput &detections,
                     const DrawDetectionsOptions &options)
{
    if (!output) {
        return;
    }

    auto image_size = detections.source_image_size;
    output->create(image_size, CV_8UC3);

    //! Draw each detected object and its keypoints
    for (const auto &det : detections.objects) {
        cv::Scalar color = options.default_box_color;

        if (options.colorization_mode == DrawDetectionsOptions::ColorizationMode::ClassId) {
            color = cv::Scalar(det.class_id * 10 % 256, det.class_id * 20 % 256, det.class_id * 30 % 256);
        } else if (options.colorization_mode == DrawDetectionsOptions::ColorizationMode::Random) {
            color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);
        } else if (det.vis_color.has_value()) {
            color = det.vis_color.value();
        }

        //! Draw bounding box if enabled
        if (options.draw_bboxes) {
            cv::Rect bbox(det.xywh[0], det.xywh[1], det.xywh[2], det.xywh[3]);
            cv::rectangle(*output, bbox, color, options.box_thickness);
        }

        //! Draw keypoints if enabled
        if (options.draw_keypoints) {
            for (const auto &kp : det.keypoints) {
                cv::circle(*output, cv::Point(kp.xy[0], kp.xy[1]), options.keypoint_radius, color, -1);
            }
        }
    }
}
} // namespace redoxi_works::dnn_models::visualizations
