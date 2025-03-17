#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelTypes.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8Preprocessor.hpp>
#include <array>

namespace redoxi_works::inference::yolo8
{

class Postprocessor
{
  public:
    virtual ~Postprocessor() = default;
    virtual void init(const PostprocessorConfig &config);

    // batch version of the postprocess function
    // by default, just iterates over the batch and calls the single image version
    virtual void postprocess(
        SingleImageOutput::List *output_result,
        const float *model_output_batch_values_numboxes,
        const std::array<int64_t, 3> &model_output_shape,
        const ImagePreprocessInfo::List &preprocess_info) const;

  protected:
    // for a single image
    // model_output_values_numboxes is a pointer to the tensor data,
    // the shape is (num_values, num_boxes),
    // num_values depends on the model type, typically (x,y,w,h,class_score1, others...)
    virtual void postprocess(
        SingleImageOutput *output_result,
        const float *model_output_values_numboxes,
        const std::array<int64_t, 2> &model_output_shape,
        const ImagePreprocessInfo &preprocess_info) const = 0;

  protected:
    std::shared_ptr<PostprocessorConfig> m_config;
};

} // namespace redoxi_works::inference::yolo8
