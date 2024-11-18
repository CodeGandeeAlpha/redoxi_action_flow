#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_dnn_models/Yolo8Preprocessor.hpp>

namespace redoxi_works::inference::yolo8
{

class Yolo8PostprocessorConfig
{
  public:
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
    std::vector<DetectedObject> objects;
};

using BatchOutput = std::vector<SingleImageOutput>;

class Yolo8Postprocessor
{
  public:
    virtual ~Yolo8Postprocessor() = default;
    virtual void init(std::shared_ptr<Yolo8PostprocessorConfig> config);

    virtual void postprocess(
        BatchOutput *result,
        const float *output_tensor_batch_values_numboxes,
        const ImagePreprocessInfo &preprocess_info,
        const Yolo8PostprocessorConfig &config) const;
};

} // namespace redoxi_works::inference::yolo8
