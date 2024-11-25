#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_inference/default_impl.hpp>
#include <json_struct/json_struct.h>
#include <optional>

namespace redoxi_works::inference::yolo8
{

class Yolo8ModelConfig : public defaults::DefaultKeyValueStore
{
  public:
    using Ptr = std::shared_ptr<Yolo8ModelConfig>;
    using ConstPtr = std::shared_ptr<const Yolo8ModelConfig>;

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

    // compare two Yolo8ModelConfig objects
    bool operator==(const Yolo8ModelConfig &other) const
    {
        return m_model_path == other.m_model_path &&
               m_device_type == other.m_device_type &&
               m_device_index == other.m_device_index;
    }

    //! Compare operator for use in maps/sets
    bool operator<(const Yolo8ModelConfig &other) const
    {
        if (m_model_path != other.m_model_path) {
            return m_model_path < other.m_model_path;
        }
        if (m_device_type != other.m_device_type) {
            return m_device_type < other.m_device_type;
        }
        return m_device_index < other.m_device_index;
    }

  protected:
    std::string m_model_path;
    std::string m_device_type = common_device_types::CPU;
    int64_t m_device_index = 0;

  public:
    JS_OBJECT(JS_MEMBER_WITH_NAME(m_model_path, "model_path"),
              JS_MEMBER_WITH_NAME(m_device_type, "device_type"),
              JS_MEMBER_WITH_NAME(m_device_index, "device_index"));
};

class PostprocessorConfig
{
  public:
    // negative value to mean no threshold
    float conf_threshold = 0.25;
    float iou_threshold = 0.45;

    // output class selections, if not set, all classes are selected
    // the key is the class id, the value is the class name
    std::optional<std::map<int64_t, std::string>> selected_classes;
};

} // namespace redoxi_works::inference::yolo8
