#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_inference/default_impl.hpp>

namespace redoxi_works::inference::yolo8
{

class Yolo8ModelConfig : public defaults::DefaultKeyValueStore
{
  public:
    Yolo8ModelConfig()
    {
        namespace cmkeys = common_config_keys;
        register_key(KeyInfo{cmkeys::ModelPath,
                             "string", "The path to the model file"},
                     &m_model_path);
        register_key(KeyInfo{cmkeys::DeviceType,
                             "string", "The type of the device"},
                     &m_device_type);
        register_key(KeyInfo{cmkeys::DeviceIndex,
                             "int64", "The index of the device"},
                     &m_device_index);
    }

  protected:
    std::string m_model_path;
    std::string m_device_type = common_device_types::CPU;
    int64_t m_device_index = 0;
};

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

} // namespace redoxi_works::inference::yolo8
