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
    RknnModelPortInfo()
    {
        // default to float32
        m_dtype = DataType_t::RKNN_TENSOR_FLOAT32;
        m_dtype_str = "float32";
    }

    virtual ~RknnModelPortInfo() = default;

    size_t get_index() const
    {
        return m_index;
    }

  public:
    rknn_tensor_attr rknn_attr;      // the underlying rknn tensor attribute
    DataType_t intrinsic_dtype;      // the intrinsic rknn data type of the port
    std::string intrinsic_dtype_str; // the intrinsic data type of the port represented as string

  protected:
    DataType_t m_dtype;
    size_t m_index{0}; // index of the port in the model
};
} // namespace redoxi_works::inference::rknn