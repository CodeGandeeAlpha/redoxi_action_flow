#pragma once

#include <rknn_api.h>
#include <redoxi_inference/redoxi_inference.hpp>

namespace redoxi_works::inference::rknn
{

class RknnModelInference;

//! A class to hold the metadata of an inference port
class RknnModelPortInfo : public ModelPortInfo
{
    friend RknnModelInference;

  public:
    using DataType_t = rknn_tensor_type;
    using Ptr = std::shared_ptr<RknnModelPortInfo>;
    using ConstPtr = std::shared_ptr<const RknnModelPortInfo>;
    using PtrMap = std::map<std::string, Ptr>;
    using ConstPtrMap = std::map<std::string, ConstPtr>;

  public:
    RknnModelPortInfo() = default;
    virtual ~RknnModelPortInfo() = default;

  protected:
    DataType_t m_dtype = DataType_t::RKNN_TENSOR_FLOAT16; // default to float16
    size_t m_index{0};                                    // index of the port in the model
};
} // namespace redoxi_works::inference::rknn