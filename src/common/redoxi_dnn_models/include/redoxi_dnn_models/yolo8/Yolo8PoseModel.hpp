#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_dnn_models/detection_types.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelBase.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8Postprocessor.hpp>

namespace redoxi_works::inference::yolo8
{
// using namespace redoxi_works::inference::detection::types;
namespace det_types = redoxi_works::inference::detection::types;

class Yolo8PoseModel : public Yolo8ModelBase
{
  public:
    // Yolo8Pose specific

    // postprocess the model output, and get the detections
    virtual det_types::SingleImageOutput::List get_output_detections(
        InferenceInOutData::Ptr model_inout_data,
        const OutputConfig_t &config) const override;

    // get the keypoint connections
    // output[i]=(u,v) means their is a connection between keypoint[u] and keypoint[v]
    static const std::vector<std::pair<int, int>> &get_keypoint_connections();
};

class PoseModelPostprocessor : public Postprocessor
{
  public:
    // pose model is only for human class
    inline static constexpr int HumanClassId = 0;
    using Postprocessor::postprocess;

  protected:
    // for a single image
    // model_output_values_numboxes is a pointer to the tensor data,
    // the shape is (num_values, num_boxes),
    // where num_values is (x,y,w,h,score, kp1_x, kp1_y, kp1_score, kp2_x, kp2_y, kp2_score, ...)
    virtual void postprocess(
        det_types::SingleImageOutput *output_result,
        const float *model_output_values_numboxes,
        const std::array<int64_t, 2> &model_output_shape,
        const det_types::ImagePreprocessInfo &preprocess_info) const override;
};
} // namespace redoxi_works::inference::yolo8
