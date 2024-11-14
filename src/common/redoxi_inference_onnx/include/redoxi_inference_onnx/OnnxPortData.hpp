#pragma once

#include <redoxi_inference_onnx/redoxi_inference_onnx.hpp>
#include <redoxi_inference_onnx/OnnxPortInfo.hpp>
#include <variant>
#include <algorithm>
#include <fmt/core.h>

namespace redoxi_works::inference::onnx
{

//! A class to hold the tensor data and the corresponding onnx tensor
template <typename T>
struct MappedTensorData {
    std::shared_ptr<std::vector<T>> data;
    std::shared_ptr<Ort::Value> onnx_tensor;
    std::shared_ptr<Ort::MemoryInfo> onnx_memory_info;
    std::vector<int64_t> shape;

    MappedTensorData() = default;
    MappedTensorData(const std::vector<int64_t> &shape)
    {
        init(shape);
    }

    virtual ~MappedTensorData() = default;

    //! Check if any dimension is dynamic
    bool has_dynamic_dims() const
    {
        return std::any_of(shape.begin(), shape.end(), [](int64_t dim) { return dim < 0; });
    }

    //! Check if data is allocated
    bool has_data() const
    {
        return data != nullptr;
    }

    //! Initialize all internals based on the shape
    //! If you change the shape, you need to call this function again
    //! For dynamic dimensions, data will not be allocated
    void init(const std::vector<int64_t> &shape)
    {
        if (shape.empty()) {
            // no shape, nothing to do
            return;
        }
        this->shape = shape;
        this->onnx_memory_info = std::make_shared<Ort::MemoryInfo>(
            Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault));

        // dynamic dimensions cannot be preallocated, just skip
        if (has_dynamic_dims()) {
            this->data.reset();
            this->onnx_tensor.reset();
            return;
        }

        // allocate data
        auto num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());
        this->data = std::make_shared<std::vector<T>>(num_elements);
        this->onnx_tensor = std::make_shared<Ort::Value>(Ort::Value::CreateTensor<T>(
            *onnx_memory_info, data->data(), data->size(),
            shape.data(), shape.size()));
    }
};

using MappedTensorData_f32 = MappedTensorData<float>;
using MappedTensorData_u8 = MappedTensorData<uint8_t>;

class OnnxModelInference;
class OnnxInferenceInOutData;

//! A class to hold the tensor data ready to be used with a port
class OnnxPortData : public ModelPortData
{
    friend class OnnxModelInference;
    friend class OnnxInferenceInOutData;

  public:
    using Ptr = std::shared_ptr<OnnxPortData>;
    using ConstPtr = std::shared_ptr<const OnnxPortData>;

  public:
    OnnxPortData();
    virtual ~OnnxPortData();

    // get information of the port
    virtual ModelPortInfo::ConstPtr get_port_info() const override
    {
        return m_port_info;
    }

    virtual int set_tensor_data(const float *data, std::vector<int64_t> shape) override
    {
        if (shape != m_shape) {
            //! Shape mismatch error
            return -1;
        }

        if (!std::holds_alternative<MappedTensorData_f32>(m_tensor_data)) {
            //! Not holding float data
            return -1;
        } else {
            std::get<MappedTensorData_f32>(m_tensor_data).data->assign(data, data + m_shape.size());
            return 0;
        }
    }
    virtual int set_tensor_data(const uint8_t *data, std::vector<int64_t> shape) override
    {
        if (shape != m_shape) {
            //! Shape mismatch error
            return -1;
        }

        if (!std::holds_alternative<MappedTensorData_u8>(m_tensor_data)) {
            //! Not holding uint8_t data
            return -1;
        } else {
            std::get<MappedTensorData_u8>(m_tensor_data).data->assign(data, data + m_shape.size());
            return 0;
        }
    }

    virtual int get_tensor_data(const float **output_tensor) const override
    {
        if (!std::holds_alternative<MappedTensorData_f32>(m_tensor_data)) {
            //! No data available
            return -1;
        }

        auto &tensor_data_f32 = std::get<MappedTensorData_f32>(m_tensor_data);
        if (tensor_data_f32.data->empty()) {
            //! No data available
            return -1;
        }

        *output_tensor = tensor_data_f32.data->data();
        return 0;
    }

    virtual int get_tensor_data(float **output_tensor) override
    {
        return get_tensor_data(const_cast<const float **>(output_tensor));
    }

    virtual int get_tensor_data(const uint8_t **output_tensor) const override
    {
        if (!std::holds_alternative<MappedTensorData_u8>(m_tensor_data)) {
            //! No data available
            return -1;
        }

        auto &tensor_data_u8 = std::get<MappedTensorData_u8>(m_tensor_data);
        if (tensor_data_u8.data->empty()) {
            //! No data available
            return -1;
        }

        *output_tensor = tensor_data_u8.data->data();
        return 0;
    }

    virtual int get_tensor_data(uint8_t **output_tensor) override
    {
        return get_tensor_data(const_cast<const uint8_t **>(output_tensor));
    }

    virtual std::vector<int64_t> get_shape() const override
    {
        return m_shape;
    }

    virtual std::string get_dtype_str() const override
    {
        if (m_port_info) {
            return m_port_info->get_dtype_str();
        }
        return "";
    }

    //! Set the shape
    //! If the shape is not compatible with the port shape, it will raise an error
    //! If the shape is compatible, it will allocate the data
    //! Note that if dynamic dimensions are present in shape, it is considered ok, but data will not be allocated
    virtual int set_shape(const std::vector<int64_t> &shape)
    {
        // you must have port info to set the shape
        if (!m_port_info) {
            RDX_RAISE_ERROR("[f={}] Port info not set", __func__);
            return -1;
        }

        // check if the shape is compatible with the port shape
        const auto &port_shape = m_port_info->get_shape();
        bool ok = is_shape_compatible(shape, port_shape);
        if (!ok) {
            std::string shape_str = fmt::format("{}", fmt::join(shape, ", "));
            std::string port_shape_str = fmt::format("{}", fmt::join(port_shape, ", "));
            RDX_RAISE_ERROR("[f={}] Shape mismatch: {} vs {}", __func__, shape_str, port_shape_str);
            return -1;
        }

        // set the shape
        m_shape = shape;

        // allocate data
        {
            bool ok = _allocate_data() == 0;
            if (!ok) {
                RDX_RAISE_ERROR("[f={}] Failed to allocate data", __func__);
                return -1;
            }
        }

        return 0;
    }

    //! Set the port info and shape, the shape should be more specific than the port shape
    //! If shape is provided, it will check if the shape is compatible with the port shape
    //! If not, it will raise an error
    virtual int init(OnnxModelPortInfo::Ptr port_info,
                     std::optional<std::vector<int64_t>> shape = std::nullopt)
    {
        m_port_info = port_info;
        if (shape.has_value()) {
            return set_shape(shape.value());
        } else {
            return set_shape(port_info->get_shape());
        }
    }

  protected:
    //! Initialize all internals based on the port info and shape
    //! If you change the shape, you need to call this function again
    virtual int _allocate_data()
    {
        // do we have port info?
        if (!m_port_info) {
            RDX_RAISE_ERROR("[f={}] Port info not found", __func__);
            return -1;
        }

        // do we have shape?
        if (m_shape.empty()) {
            RDX_RAISE_ERROR("[f={}] Shape not set", __func__);
            return -1;
        }

        // allocate data
        auto dtype_str = get_dtype_str();
        if (dtype_str == "float32") {
            m_tensor_data = MappedTensorData_f32(m_shape);
        } else if (dtype_str == "uint8") {
            m_tensor_data = MappedTensorData_u8(m_shape);
        } else {
            RDX_RAISE_ERROR("Unsupported data type: {}", dtype_str);
            return -1;
        }

        return 0;
    }

  protected:
    OnnxModelPortInfo::Ptr m_port_info;
    std::variant<MappedTensorData_f32,
                 MappedTensorData_u8>
        m_tensor_data;
    std::vector<int64_t> m_shape;
};

} // namespace redoxi_works::inference::onnx
