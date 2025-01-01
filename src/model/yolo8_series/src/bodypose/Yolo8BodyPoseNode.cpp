#include <yolo8_series/_pch.hpp>
#include <yolo8_series/bodypose/Yolo8BodyPoseNode.hpp>

namespace redoxi_works::model_nodes::yolo8
{
int Yolo8BodyPoseNode::_process_detection_result(typename ByImageRequest::OutputSourceData_t *output_source_data,
                                                 const DetectionResult_t &det_result,
                                                 const typename ByImageRequest::InputSourceData_t &source_data,
                                                 const cv::Mat &input_image)
{
    Yolo8BaseNode_t::_process_detection_result(output_source_data, det_result, source_data, input_image);

    // add keypoint connections
    auto keypoint_connections = bodypose::InferenceModel::get_keypoint_connections();

    // flatten the keypoint connections
    std::vector<int64_t> keypoint_edges;
    for (auto &connection : keypoint_connections) {
        keypoint_edges.push_back(connection.first);
        keypoint_edges.push_back(connection.second);
    }

    // assign the keypoint connections to the detections
    if (keypoint_edges.size() > 0) {
        for (auto &detection : output_source_data->detections) {
            auto &keypoints = detection.keypoints;
            keypoints.keypoint_edges = keypoint_edges;
        }
    }

    return 0;
}

int Yolo8BodyPoseNode::_process_detection_result(typename ByDetectionRequest::InputActionResult_t *output_action_result,
                                                 const DetectionResult_t &det_result,
                                                 const typename ByDetectionRequest::InputSourceData_t &source_data,
                                                 const cv::Mat &input_image)
{
    Yolo8BaseNode_t::_process_detection_result(output_action_result, det_result, source_data, input_image);

    // add keypoint connections
    auto keypoint_connections = bodypose::InferenceModel::get_keypoint_connections();

    // flatten the keypoint connections
    std::vector<int64_t> keypoint_edges;
    for (auto &connection : keypoint_connections) {
        keypoint_edges.push_back(connection.first);
        keypoint_edges.push_back(connection.second);
    }

    // assign the keypoint connections to the detections
    if (keypoint_edges.size() > 0) {
        for (auto &detection : output_action_result->detections) {
            auto &keypoints = detection.keypoints;
            keypoints.keypoint_edges = keypoint_edges;
        }
    }

    return 0;
}

void Yolo8BodyPoseNode::_draw_visualization(cv::Mat &canvas, const DetectionResult_t &detections)
{
    Yolo8BaseNode_t::_draw_visualization(canvas, detections);

    // also draw the keypoint connections
    auto keypoint_connections = bodypose::InferenceModel::get_keypoint_connections();
    for (const auto &det : detections.objects) {
        auto &keypoints = det.keypoints;
        for (const auto &connection : keypoint_connections) {
            auto p1 = keypoints[connection.first];
            auto p2 = keypoints[connection.second];
            cv::line(canvas, cv::Point(p1.xy[0], p1.xy[1]), cv::Point(p2.xy[0], p2.xy[1]), cv::Scalar(0, 255, 0), 2);
        }
    }
}
} // namespace redoxi_works::model_nodes::yolo8
