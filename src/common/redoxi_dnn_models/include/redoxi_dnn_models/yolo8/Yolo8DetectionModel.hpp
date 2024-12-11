#pragma once

#include <unordered_map>
#include <string>
#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelBase.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelTypes.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8Postprocessor.hpp>
namespace redoxi_works::inference::yolo8
{

class Yolo8DetectionModel : public Yolo8ModelBase
{
  public:
    // postprocess the model output, and get the detections
    virtual std::vector<SingleImageOutput> get_output_detections(
        InferenceInOutData::Ptr model_inout_data,
        const OutputConfig_t &config) const override;
};

class DetectionModelPostprocessor : public Postprocessor
{
  public:
    using Postprocessor::postprocess;

  protected:
    // for a single image
    // model_output_values_numboxes is a pointer to the tensor data,
    // the shape is (num_values, num_boxes),
    // where num_values is (x,y,w,h,score, others...)
    virtual void postprocess(
        SingleImageOutput *output_result,
        const float *model_output_values_numboxes,
        const std::array<int64_t, 2> &model_output_shape,
        const ImagePreprocessInfo &preprocess_info) const override;
};

} // namespace redoxi_works::inference::yolo8
