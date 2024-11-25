#include <pluginlib/class_loader.hpp>

#include <redoxi_dnn_models/yolo8/Yolo8PoseModel.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8Preprocessor.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8Postprocessor.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelBaseImpl.hpp>

namespace redoxi_works::inference::yolo8
{

det_types::SingleImageOutput::List
    Yolo8PoseModel::get_output_detections(
        InferenceInOutData::Ptr model_inout_data,
        const OutputConfig_t &config) const
{
    // get the output port data
    auto output_port_name = m_model_output_info->get_name();
    auto port_data = model_inout_data->get_output_port_data(output_port_name);
    if (!port_data) {
        RDX_RAISE_ERROR("Output port data not found for port: {}", output_port_name);
    }
    if (port_data->get_dtype_str() != "float32") {
        RDX_RAISE_ERROR("Unsupported model output dtype: {}, required float32", port_data->get_dtype_str());
    }

    // expected output tensor (batch_size, 4+num_keypoints*3, num_objects)
    auto tensor_shape = port_data->get_shape();
    const float *tensor_data = nullptr;
    port_data->get_tensor_data(&tensor_data);

    // check tensor shape
    if (tensor_shape.size() != 3) {
        RDX_RAISE_ERROR("Invalid output tensor shape: expected 3 dimensions, got {}", tensor_shape.size());
    }
    RDX_INFO_DEV(nullptr, __func__, "Output tensor shape: ({},{},{})", tensor_shape[0], tensor_shape[1], tensor_shape[2]);

    // get preprocess info
    auto any_data = model_inout_data->get_any_data(Impl::PreprocessInfoKey);
    if (!any_data) {
        RDX_RAISE_ERROR("Preprocess info not found");
    }
    auto preprocess_info = std::any_cast<ImagePreprocessInfo::List>(*any_data);

    PoseModelPostprocessor postprocessor;
    postprocessor.init(config);

    SingleImageOutput::List outputs;
    postprocessor.postprocess(&outputs, tensor_data,
                              std::array<int64_t, 3>{tensor_shape[0], tensor_shape[1], tensor_shape[2]},
                              preprocess_info);
    return outputs;
}

const std::vector<std::pair<int, int>> &
    Yolo8PoseModel::get_keypoint_connections()
{
    static const std::vector<std::pair<int, int>> connections = {
        {15, 13}, {13, 11}, {16, 14}, {14, 12}, {11, 12}, {5, 11}, {6, 12}, {5, 6}, {5, 7}, {6, 8}, {7, 9}, {8, 10}, {1, 2}, {0, 1}, {0, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}};
    return connections;
}

// for a single image
void PoseModelPostprocessor::postprocess(
    det_types::SingleImageOutput *output_result,
    const float *model_output_values_numboxes,
    const std::array<int64_t, 2> &model_output_shape,
    const det_types::ImagePreprocessInfo &preprocess_info) const
{
    // model_output_batch_values_numboxes is a pointer to the tensor data,
    // the shape is (num_values, num_boxes), given by model_output_shape
    // where the first dimension is (x,y,w,h,score, kp1_x, kp1_y, kp1_score, kp2_x, kp2_y, kp2_score, ...)
    // and the second dimension is the boxes

    // nothing to do if output_result is not provided
    if (!output_result) {
        return;
    }

    // config is not set?
    if (!m_config) {
        throw std::runtime_error("PoseModelPostprocessor config is not set");
    }

    // get dimensions
    int64_t num_values = model_output_shape[0];
    int64_t num_objects = model_output_shape[1];

    // each object has 4 bbox values (x_center,y_center,width,height) + score + keypoints (x,y,score)
    int64_t num_keypoints = (num_values - 5) / 3;

    //! Process each potential object, convert to the format to (num_boxes, num_values), easier to iterate over boxes
    cv::Mat _object_data(num_values, num_objects, CV_32FC1, const_cast<float *>(model_output_values_numboxes));
    cv::Mat object_data;
    cv::transpose(_object_data, object_data);

    SingleImageOutput final_output;
    auto &detections = final_output.objects;

    for (int64_t obj = 0; obj < num_objects; obj++) {
        //! Get row data for this object
        const float *obj_data = object_data.ptr<float>(obj);

        //! Get confidence score
        float score = obj_data[4];

        //! Skip if below confidence threshold
        if (m_config->conf_threshold >= 0.0f && score < m_config->conf_threshold) {
            continue;
        }

        //! Extract bounding box
        float x_center = obj_data[0];
        float y_center = obj_data[1];
        float width = obj_data[2];
        float height = obj_data[3];

        //! Create detection object
        DetectedObject det;
        det.xywh = {x_center - width / 2, y_center - height / 2, width, height};
        det.score = score;

        // this model only detects human
        det.class_id = HumanClassId;

        //! Extract keypoints
        det.keypoints.resize(num_keypoints);
        for (int64_t k = 0; k < num_keypoints; k++) {
            int64_t kp_offset = 5 + k * 3;
            det.keypoints[k].xy = {obj_data[kp_offset], obj_data[kp_offset + 1]};
            det.keypoints[k].score = obj_data[kp_offset + 2];
        }

        detections.push_back(std::move(det));
    }

    {
        auto nms_threshold = m_config->iou_threshold;
        auto conf_threshold = m_config->conf_threshold;
        if (conf_threshold < 0.0f) {
            conf_threshold = 0.0f;
        }

        if (nms_threshold > 0.0f) {
            //! Convert detections to format needed for NMS
            std::vector<cv::Rect2d> boxes;
            std::vector<float> scores;
            std::vector<int> indices;
            boxes.reserve(detections.size());
            scores.reserve(detections.size());

            for (const auto &det : detections) {
                boxes.emplace_back(det.xywh[0], det.xywh[1], det.xywh[2], det.xywh[3]);
                scores.push_back(det.score);
            }

            //! Apply NMS
            cv::dnn::NMSBoxes(boxes, scores, conf_threshold, nms_threshold, indices);

            //! Keep only detections that passed NMS
            std::vector<DetectedObject> filtered_detections;
            filtered_detections.reserve(indices.size());
            for (int idx : indices) {
                filtered_detections.push_back(std::move(detections[idx]));
            }
            detections = std::move(filtered_detections);
        }
    }

    // transform all coordinates from model input space to source image space
    auto dx_model = -preprocess_info.roi_in_model_input_image.x;
    auto dy_model = -preprocess_info.roi_in_model_input_image.y;
    auto scale_x = preprocess_info.roi_in_source_image.width / (float)preprocess_info.roi_in_model_input_image.width;
    auto scale_y = preprocess_info.roi_in_source_image.height / (float)preprocess_info.roi_in_model_input_image.height;
    auto dx_source = preprocess_info.roi_in_source_image.x;
    auto dy_source = preprocess_info.roi_in_source_image.y;

    for (auto &det : detections) {
        det.xywh[0] = (det.xywh[0] + dx_model) * scale_x + dx_source;
        det.xywh[1] = (det.xywh[1] + dy_model) * scale_y + dy_source;
        det.xywh[2] *= scale_x;
        det.xywh[3] *= scale_y;

        // transform keypoints
        for (auto &kp : det.keypoints) {
            kp.xy[0] = (kp.xy[0] + dx_model) * scale_x + dx_source;
            kp.xy[1] = (kp.xy[1] + dy_model) * scale_y + dy_source;
        }
    }

    if (output_result) {
        *output_result = std::move(final_output);
    }
}

} // namespace redoxi_works::inference::yolo8
