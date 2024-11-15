#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
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

class OnnxModelInference;
struct OnnxModelConfig : public KeyValueStore {
    friend class OnnxModelInference;

  public:
    struct Keys {
        inline constexpr static const char *ModelPath = "model_path";
        inline constexpr static const char *ExecutionProvider = "execution_provider";
        inline constexpr static const char *LoggingLevel = "logging_level";
        inline constexpr static const char *LogId = "log_id";
    };

    const std::vector<KeyValueStore::KeyInfo> all_keys{
        {Keys::ModelPath, "string", "The path of the ONNX model file"},
        {Keys::ExecutionProvider, "string", "The execution provider for the ONNX model"},
        {Keys::LoggingLevel, "int64", "The logging level for the ONNX model"},
        {Keys::LogId, "string", "The log id for the ONNX model"},
    };

    const std::map<std::string, std::any> name_to_variable_addr{
        {Keys::ModelPath, &model_path},
        {Keys::ExecutionProvider, &execution_provider},
        {Keys::LoggingLevel, &logging_level},
        {Keys::LogId, &log_id},
    };

  public:
    OnnxModelConfig()
    {
        // generate a random log id by default
        std::stringstream ss;
        ss << "onnx_" << std::setw(4) << std::setfill('0') << (rand() % 10000);
        std::string log_id = ss.str();
    }

    const std::vector<KeyValueStore::KeyInfo> &get_all_keys() const override
    {
        return all_keys;
    }

    int get_string(std::string *output, const std::string &key) const override
    {
        auto it = name_to_variable_addr.find(key);
        if (it != name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<std::string *>(it->second);
                if (output && variable_addr) {
                    *output = *variable_addr;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        return -1;
    }

    int set_string(const std::string &key, const std::string &value) override
    {
        auto it = name_to_variable_addr.find(key);
        if (it != name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<std::string *>(it->second);
                if (variable_addr) {
                    *variable_addr = value;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        //! Key not found
        return -1;
    }

    int get_int(int64_t *output, const std::string &key) const override
    {
        auto it = name_to_variable_addr.find(key);
        if (it != name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<int64_t *>(it->second);
                if (output && variable_addr) {
                    *output = *variable_addr;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        return -1;
    }
    int set_int(const std::string &key, int64_t value) override
    {
        auto it = name_to_variable_addr.find(key);
        if (it != name_to_variable_addr.end()) {
            try {
                auto variable_addr = std::any_cast<int64_t *>(it->second);
                if (variable_addr) {
                    *variable_addr = value;
                    return 0;
                }
            } catch (const std::bad_any_cast &e) {
                //! Handle any_cast failure
                return -1;
            }
        }
        return -1;
    }

    int get_double(double *output, const std::string &key) const override
    {
        (void)output;
        (void)key;
        return -1;
    }
    int set_double(const std::string &key, double value) override
    {
        (void)key;
        (void)value;
        return -1;
    }

    bool has_key(const std::string &key) const override
    {
        if (key == Keys::ModelPath || key == Keys::ExecutionProvider) {
            return true;
        }
        return false;
    }

  protected:
    std::string model_path;
    std::string execution_provider = onnx_ep_names::CPU;
    int64_t logging_level = static_cast<int64_t>(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING);
    std::string log_id;
};

} // namespace redoxi_works::inference::onnx
