#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelTypes.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8Preprocessor.hpp>
#include <array>

namespace redoxi_works::inference::yolo8
{

class PostprocessorConfig
{
  public:
    // negative value to mean no threshold
    float conf_threshold = 0.25;
    float iou_threshold = 0.45;
};

// keypoint in the image
struct Keypoint {
    std::array<float, 2> xy = {0, 0};
    float score = 0;
};

// detected object in the image
struct DetectedObject {
    int64_t class_id = 0;
    std::array<float, 4> xywh = {0, 0, 0, 0};
    float score = 0;
    std::vector<Keypoint> keypoints;
};

// output of the model for a single image
struct SingleImageOutput {
    using List = std::vector<SingleImageOutput>;
    std::vector<DetectedObject> objects;
};

class PoseModelPostprocessor
{
  public:
    inline static constexpr int HumanClassId = 0;

    virtual ~PoseModelPostprocessor() = default;
    virtual void init(const PostprocessorConfig &config);

    // batch version of the postprocess function
    virtual void postprocess(
        SingleImageOutput::List *output_result,
        const float *model_output_batch_values_numboxes,
        const std::array<int64_t, 3> &model_output_shape,
        const ImagePreprocessInfo::List &preprocess_info) const;

  protected:
    // for a single image
    // model_output_values_numboxes is a pointer to the tensor data,
    // the shape is (num_values, num_boxes),
    // where num_values is (x,y,w,h,score, kp1_x, kp1_y, kp1_score, kp2_x, kp2_y, kp2_score, ...)
    virtual void postprocess(
        SingleImageOutput *output_result,
        const float *model_output_values_numboxes,
        const std::array<int64_t, 2> &model_output_shape,
        const ImagePreprocessInfo &preprocess_info) const;

  protected:
    std::shared_ptr<PostprocessorConfig> m_config;
};

} // namespace redoxi_works::inference::yolo8
