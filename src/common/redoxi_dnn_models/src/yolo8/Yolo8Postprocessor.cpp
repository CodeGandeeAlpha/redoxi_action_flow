#include <redoxi_dnn_models/yolo8/Yolo8Postprocessor.hpp>
#include <opencv2/dnn/dnn.hpp>

namespace redoxi_works::inference::yolo8
{

void PoseModelPostprocessor::init(const PostprocessorConfig &config)
{
    m_config = std::make_shared<PostprocessorConfig>(config);
}

void PoseModelPostprocessor::postprocess(
    SingleImageOutput::List *output_result,
    const float *model_output_batch_values_numboxes,
    const std::array<int64_t, 3> &model_output_shape,
    const ImagePreprocessInfo::List &preprocess_info) const
{
    // nothing to do if output_result is not provided
    if (!output_result) {
        return;
    }

    // get dimensions
    int64_t batch_size = model_output_shape[0];
    int64_t num_values = model_output_shape[1];
    int64_t num_boxes = model_output_shape[2];

    // prepare output list
    output_result->resize(batch_size);

    // process each image in batch
    for (int64_t i = 0; i < batch_size; i++) {
        // get pointer to data for this image
        const float *image_data = model_output_batch_values_numboxes + i * num_values * num_boxes;

        // get preprocess info for this image
        const auto &pinfo = preprocess_info[i];

        // process single image
        std::array<int64_t, 2> single_output_shape = {num_values, num_boxes};
        postprocess(&(*output_result)[i], image_data, single_output_shape, pinfo);
    }
}

// for a single image
void PoseModelPostprocessor::postprocess(
    SingleImageOutput *output_result,
    const float *model_output_values_numboxes,
    const std::array<int64_t, 2> &model_output_shape,
    const ImagePreprocessInfo &preprocess_info) const
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
