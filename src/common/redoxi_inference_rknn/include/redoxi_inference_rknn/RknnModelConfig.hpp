#pragma once

#include <redoxi_inference/redoxi_inference.hpp>
#include <redoxi_basic_cpp/logging/ros_logging.hpp>
#include <redoxi_inference/default_impl.hpp>
#include <rknn_api.h>
#include <string>
#include <sstream>
#include <tuple>
#include <any>

namespace redoxi_works::inference::rknn
{

namespace cmkeys = redoxi_works::inference::common_config_keys;
namespace cmdev = redoxi_works::inference::common_device_types;

class RknnModelInference;
struct RknnModelConfig : public defaults::DefaultKeyValueStore {
    friend class RknnModelInference;

  public:
    using Ptr = std::shared_ptr<RknnModelConfig>;
    using ConstPtr = std::shared_ptr<const RknnModelConfig>;
    inline static constexpr const char *DefaultDeviceType = cmdev::RKNPU;

    struct Keys {
        inline static constexpr auto ModelPath = cmkeys::ModelPath;
        inline static constexpr auto DeviceType = cmkeys::DeviceType;
        inline static constexpr auto DeviceIndex = cmkeys::DeviceIndex;
    }; // namespace Keys

    RknnModelConfig()
    {
        // Register all keys
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::ModelPath);
        register_key({Keys::ModelPath, "string", "The path of the ONNX model file"}, &model_path);
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::DeviceType);
        register_key({Keys::DeviceType, "string", "The device type for the ONNX model"}, &device_type);
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::DeviceIndex);
        register_key({Keys::DeviceIndex, "int64", "The device index for the ONNX model"}, &device_index);
    }

  protected:
    std::string model_path;
    std::string device_type{DefaultDeviceType};
    int64_t device_index = 0;
};

} // namespace redoxi_works::inference::rknn
