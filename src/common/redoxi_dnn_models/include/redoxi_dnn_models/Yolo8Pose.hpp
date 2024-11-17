#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_inference/redoxi_inference.hpp>

namespace redoxi_works::inference
{

class Yolo8Pose : public RedoxiModelInference
{
  public:
    struct InitConfig {
        std::string model_path;
    };

    void init(const InitConfig &config);
};
} // namespace redoxi_works::inference
