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
        inline static constexpr auto CoreMask = "core_mask";
    }; // namespace Keys

    RknnModelConfig()
    {
        // Register all keys
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::ModelPath);
        register_key({Keys::ModelPath, "string", "The path of the RKNN model file"}, &model_path);
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::DeviceType);
        register_key({Keys::DeviceType, "string", "The device type for the RKNN model"}, &device_type);
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::DeviceIndex);

        register_key({Keys::DeviceIndex, "int64",
                      "The npu core index to use.\n"
                      "- 0,1,2 means core 0,1,2\n"
                      "- std::numeric_limits<int64_t>::max() means all cores\n"
                      "- -1 means unspecified, leave it for core mask\n"
                      "To use core mask, set core_mask property"},
                     &device_index);

        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::CoreMask);
        register_key({Keys::CoreMask, "int64",
                      "The npu core mask to use, will be used when device_index is -1.\n"
                      "This is a bitmask where:\n"
                      "- 1<<0 means core 0\n"
                      "- 1<<1 means core 1\n"
                      "- 1<<2 means core 2\n"
                      "- (1<<1)|(1<<2) means core 1 and 2\n"
                      "etc."},
                     &core_mask);
    }

  protected:
    std::string model_path;
    std::string device_type{DefaultDeviceType};
    int64_t device_index = -1; // -1 means all cores
    int64_t core_mask = 0;
};

} // namespace redoxi_works::inference::rknn
