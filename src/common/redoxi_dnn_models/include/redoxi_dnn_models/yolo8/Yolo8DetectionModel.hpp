#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelTypes.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8Postprocessor.hpp>

namespace redoxi_works::inference::yolo8
{

class Yolo8DetectionModel : public RedoxiModelInference
{
  public:
    using InitConfig_t = Yolo8ModelConfig;
    using OutputConfig_t = PostprocessorConfig;
};

} // namespace redoxi_works::inference::yolo8
