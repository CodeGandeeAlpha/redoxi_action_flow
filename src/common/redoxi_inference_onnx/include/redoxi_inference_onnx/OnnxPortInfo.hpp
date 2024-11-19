#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
#include <onnxruntime_cxx_api.h>

namespace redoxi_works::inference::onnx
{

class OnnxModelInference;

//! A class to hold the metadata of an inference port
class OnnxModelPortInfo : public ModelPortInfo
{
    friend OnnxModelInference;

  public:
    using Ptr = std::shared_ptr<OnnxModelPortInfo>;
    using ConstPtr = std::shared_ptr<const OnnxModelPortInfo>;
    using PtrMap = std::map<std::string, Ptr>;
    using ConstPtrMap = std::map<std::string, ConstPtr>;

  public:
    OnnxModelPortInfo() = default;
    virtual ~OnnxModelPortInfo() = default;

  protected:
    ONNXTensorElementDataType m_dtype;
    size_t m_index{0};
};
} // namespace redoxi_works::inference::onnx