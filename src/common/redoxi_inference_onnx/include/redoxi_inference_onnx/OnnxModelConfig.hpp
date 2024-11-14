#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
#include <string>

namespace redoxi_works::inference::onnx
{


namespace onnx_ep_names
{
    //! Execution providers of onnx runtime
    constexpr const char* CPU = "CPUExecutionProvider";
    constexpr const char* CUDA = "CUDAExecutionProvider";
    constexpr const char* TensorRT = "TensorrtExecutionProvider";
}  // namespace onnx_ep_names

namespace onnx_env_keys
{
    //! Environment keys, from which some parameters can be configured
    constexpr const char* ExecutionProvider = "RDX_ONNX_EXECUTION_PROVIDER";
}  // namespace onnx_env_keys

struct OnnxModelConfig: public KeyValueStore
{
public:
    struct Keys
    {
        inline constexpr static const char* ModelPath = "model_path";
        inline constexpr static const char* ExecutionProvider = "execution_provider";
    };

    const std::vector<KeyValueStore::KeyInfo> all_keys{
        {Keys::ModelPath, "string", "The path of the ONNX model file"},
        {Keys::ExecutionProvider, "string", "The execution provider for the ONNX model"}
    };

public:
    const std::vector<KeyValueStore::KeyInfo>& get_all_keys() const override
    {
        return all_keys;
    }
    
    int get_string(std::string *output, const std::string &key) const override
    {
        if (key == Keys::ModelPath)
        {
            if (output)
            {
                *output = model_path;
            }
            return 0;
        }
        else if (key == Keys::ExecutionProvider)
        {
            if (output)
            {
                *output = execution_provider;
            }
            return 0;
        }

        //! Key not found
        return -1;
    }

    int set_string(const std::string &key, const std::string &value) override
    {
        if (key == Keys::ModelPath)
        {
            model_path = value;
            return 0;
        }
        else if (key == Keys::ExecutionProvider)
        {
            execution_provider = value;
            return 0;
        }

        //! Key not found
        return -1;
    }

    int get_int(int64_t *output, const std::string &key) const override
    {
        (void)output;
        (void)key;
        return -1;
    }
    int set_int(const std::string &key, int64_t value) override
    {
        (void)key;
        (void)value;
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
        if (key == Keys::ModelPath || key == Keys::ExecutionProvider)
        {
            return true;
        }
        return false;
    }

public:
    std::string model_path;
    std::string execution_provider = onnx_ep_names::CPU;
};

}  // namespace redoxi_works::inference::onnx
