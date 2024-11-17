#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_inference/default_impl.hpp>

namespace redoxi_works::inference
{

class Yolo8PoseConfig : public defaults::DefaultKeyValueStore
{
  public:
    Yolo8PoseConfig()
    {
        namespace cmkeys = common_config_keys;
        register_key(KeyInfo{cmkeys::ModelPath,
                             "string", "The path to the model file"},
                     &m_model_path);
        register_key(KeyInfo{cmkeys::DeviceType,
                             "string", "The type of the device"},
                     &m_device_type);
        register_key(KeyInfo{cmkeys::DeviceIndex,
                             "int", "The index of the device"},
                     &m_device_index);
    }

  protected:
    std::string m_model_path;
    std::string m_device_type = common_device_types::CPU;
    int m_device_index = 0;
};

} // namespace redoxi_works::inference
