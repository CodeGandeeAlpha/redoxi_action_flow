#pragma once

#include <yolo8_series/bodypose/Yolo8BodyPoseTypes.hpp>
#include <yolo8_series/base/Yolo8BaseNode.hpp>

namespace redoxi_works::model_nodes::yolo8
{
class Yolo8BodyPoseNode : public Yolo8BaseNode<bodypose::InferenceModel>
{
  public:
    using Yolo8BaseNode_t = Yolo8BaseNode<bodypose::InferenceModel>;
    using Yolo8BaseNode_t::Yolo8BaseNode_t;

    int _process_detection_result(typename ByImageRequest::OutputSourceData_t *output_source_data,
                                  const DetectionResult_t &det_result,
                                  const typename ByImageRequest::InputSourceData_t &source_data,
                                  const cv::Mat &input_image) override;

    int _process_detection_result(typename ByDetectionRequest::InputActionResult_t *output_action_result,
                                  const DetectionResult_t &det_result,
                                  const typename ByDetectionRequest::InputSourceData_t &source_data,
                                  const cv::Mat &input_image) override;

    void _draw_visualization(cv::Mat &canvas, const DetectionResult_t &detections) override;
};
} // namespace redoxi_works::model_nodes::yolo8
