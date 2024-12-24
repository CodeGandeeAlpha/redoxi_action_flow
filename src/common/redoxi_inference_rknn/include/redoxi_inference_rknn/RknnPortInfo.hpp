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
    using TensorAttributes_t = rknn_tensor_attr;
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

    //! Get the current shape of the port, which is already being used by the model
    //! If empty, it means the shape is not determined yet
    std::vector<int64_t> get_current_shape() const;

    //! Get the native shape of the port, which is the shape of the port when the model is exported
    std::vector<int64_t> get_native_shape() const;

    //! Get the default shape of the port, which is the shape of the port when the model is exported
    //! If empty, it means the shape is not determined yet
    std::vector<int64_t> get_default_shape() const;

    //! Get the intrinsic data type of the port represented as string
    std::string get_intrinsic_dtype_str() const;

    //! Get the intrinsic data type of the port
    DataType_t get_intrinsic_dtype() const;

    //! Get the tensor attributes of the port, which is native when exported
    TensorAttributes_t get_native_tensor_attributes() const;

    //! Get the default attributes of the port, which is rknn returned by default
    TensorAttributes_t get_default_tensor_attributes() const;

    //! Get the current attributes of the port, already being used by the model
    TensorAttributes_t get_current_tensor_attributes() const;

  public:
    std::vector<std::vector<int64_t>> supported_shapes; // the supported shapes of the port, useful when dynamic shape is supported by the model
    const rknn_context *context = nullptr;              // the context of the port

  protected:
    DataType_t m_dtype;
    size_t m_index{0}; // index of the port in the model
};
} // namespace redoxi_works::inference::rknn