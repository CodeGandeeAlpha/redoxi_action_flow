#pragma once

#include <redoxi_dnn_models/yolo8/Yolo8PoseModel.hpp>

namespace redoxi_works::inference::yolo8
{

//! base class for rknn yolo8 models, which used with rknn_inference_rknn
class Yolo8RknnPoseModel : public Yolo8PoseModel
{
  public:
    virtual ~Yolo8RknnPoseModel() = default;
    int open(KeyValueStore::Ptr params) override;
    int do_inference(InferenceInOutData::Ptr inout_data) override;

  protected:
    // native output ports, not exposed to the user
    ModelPortInfo::ConstPtr m_native_model_output_info;
};

} // namespace redoxi_works::inference::yolo8
