#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
#include <redoxi_inference/default_impl.hpp>
#include <onnxruntime_cxx_api.h>
#include <string>
#include <sstream>
#include <tuple>
#include <any>

namespace redoxi_works::inference::onnx
{


namespace onnx_ep_names
{
//! Execution providers of onnx runtime
constexpr const char *CPU = "CPUExecutionProvider";
constexpr const char *CUDA = "CUDAExecutionProvider";
constexpr const char *TensorRT = "TensorrtExecutionProvider";
} // namespace onnx_ep_names

namespace onnx_env_keys
{
//! Environment keys, from which some parameters can be configured
constexpr const char *ExecutionProvider = "RDX_ONNX_EXECUTION_PROVIDER";
} // namespace onnx_env_keys

namespace cmkeys = redoxi_works::inference::common_config_keys;
namespace cmdev = redoxi_works::inference::common_device_types;

class OnnxModelInference;
struct OnnxModelConfig : public redoxi_works::inference::defaults::DefaultKeyValueStore {
    friend class OnnxModelInference;

  public:
    struct Keys {
        inline static constexpr auto ModelPath = cmkeys::ModelPath;
        inline static constexpr auto DeviceType = cmkeys::DeviceType;
        inline static constexpr auto DeviceIndex = cmkeys::DeviceIndex;

        //! Execution provider, must be compatible with the device type
        //! If not specified, the default execution provider for that device type will be used
        // inline static constexpr auto ExecutionProvider = "execution_provider";

        //! Logging level, must be compatible with the device type
        //! If not specified, the default logging level for that device type will be used
        inline static constexpr auto LoggingLevel = "logging_level";

        //! Log id, must be compatible with the device type
        //! If not specified, a random log id will be generated
        inline static constexpr auto LogId = "log_id";
    }; // namespace Keys

    OnnxModelConfig()
    {
        // generate a random log id by default
        std::stringstream ss;
        ss << "onnx_" << std::setw(4) << std::setfill('0') << (rand() % 10000);
        log_id = ss.str();

        // Register all keys
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::ModelPath);
        register_key({Keys::ModelPath, "string", "The path of the ONNX model file"}, &model_path);
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::DeviceType);
        register_key({Keys::DeviceType, "string", "The device type for the ONNX model"}, &device_type);
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::DeviceIndex);
        register_key({Keys::DeviceIndex, "int64", "The device index for the ONNX model"}, &device_index);
        // register_key({Keys::ExecutionProvider, "string", "The execution provider for the ONNX model"}, &execution_provider);
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::LoggingLevel);
        register_key({Keys::LoggingLevel, "int64", "The logging level for the ONNX model"}, &logging_level);
        RDX_INFO_DEV(nullptr, __func__, "Registering key {}", Keys::LogId);
        register_key({Keys::LogId, "string", "The log id for the ONNX model"}, &log_id);
    }

    std::string get_execution_provider() const
    {
        if (device_type.empty()) {
            return onnx_ep_names::CPU;
        }

        if (device_type == cmdev::CUDA) {
            return onnx_ep_names::CUDA;
        } else if (device_type == cmdev::CPU) {
            return onnx_ep_names::CPU;
        } else {
            throw std::invalid_argument(fmt::format("[f={}] Invalid device type: {}", __func__, device_type));
        }
    }

  protected:
    std::string model_path;
    std::string device_type;
    int64_t device_index = 0;

    // std::string execution_provider;
    int64_t logging_level = static_cast<int64_t>(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING);
    std::string log_id;
};

} // namespace redoxi_works::inference::onnx
