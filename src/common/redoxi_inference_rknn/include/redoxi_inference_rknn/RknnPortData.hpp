#pragma once

#include <redoxi_basic_cpp/logging/ros_logging.hpp>
#include <redoxi_inference_rknn/RknnPortInfo.hpp>
#include <variant>
#include <algorithm>
#include <numeric>
#include <fmt/core.h>
#include <fmt/format.h>

namespace redoxi_works::inference::rknn
{

//! A class to hold the tensor data and the corresponding onnx tensor
template <typename T>
struct MappedTensorData {
    std::shared_ptr<std::vector<T>> data;
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

        // dynamic dimensions cannot be preallocated, just skip
        if (has_dynamic_dims()) {
            this->data.reset();
            return;
        }


        // allocate data
        auto num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<int64_t>());
        this->data = std::make_shared<std::vector<T>>(num_elements);
        RDX_INFO_DEV(nullptr, __func__, false, "Tensor data initialized, shape={}, num_elements={}",
                     fmt::join(shape, ", "), num_elements);
    }
};

using MappedTensorData_f32 = MappedTensorData<float>;
using MappedTensorData_u8 = MappedTensorData<uint8_t>;

class RknnModelInference;
class RknnInferenceInOutData;

//! A class to hold the tensor data ready to be used with a port
class RknnPortData : public ModelPortData
{
    friend class RknnModelInference;
    friend class RknnInferenceInOutData;

  public:
    using Ptr = std::shared_ptr<RknnPortData>;
    using ConstPtr = std::shared_ptr<const RknnPortData>;

  public:
    // get information of the port
    virtual ModelPortInfo::ConstPtr get_port_info() const override
    {
        return m_port_info;
    }

    virtual int set_tensor_data(const float *data, std::vector<int64_t> shape) override
    {
        // check dtype compatibility first
        if (m_port_info->get_dtype_str() != "float32") {
            // dtype mismatch
            return -1;
        }

        // set shape and allocate data
        auto ret = _set_shape_and_allocate_data(shape);
        if (ret != 0) {
            return ret;
        }

        auto num_elements = std::accumulate(m_shape.begin(), m_shape.end(), 1, std::multiplies<int64_t>());
        if (!std::holds_alternative<MappedTensorData_f32>(m_tensor_data)) {
            //! Not holding float data
            return -1;
        } else {
            auto &tensor_data_f32 = std::get<MappedTensorData_f32>(m_tensor_data);
            tensor_data_f32.data->assign(data, data + num_elements);
            // RDX_INFO_DEV(nullptr, __func__, false, "Port name={}, shape={}, num_elements={}",
            //              m_port_info->get_name(), fmt::join(tensor_data_f32.shape, ", "), tensor_data_f32.data->size());
            //! Print first element for debugging
            if (!tensor_data_f32.data->empty()) {
                RDX_INFO_DEV(nullptr, __func__, false, "Port name={}, first element: {}",
                             m_port_info->get_name(), tensor_data_f32.data->front());
            }
            return 0;
        }
    }

    virtual int set_tensor_data(const uint8_t *data, std::vector<int64_t> shape) override
    {
        // check dtype compatibility first
        if (m_port_info->get_dtype_str() != "uint8") {
            // dtype mismatch
            return -1;
        }

        // set shape and allocate data
        auto ret = _set_shape_and_allocate_data(shape);
        if (ret != 0) {
            return ret;
        }

        auto num_elements = std::accumulate(m_shape.begin(), m_shape.end(), 1, std::multiplies<int64_t>());
        if (!std::holds_alternative<MappedTensorData_u8>(m_tensor_data)) {
            //! Not holding uint8_t data
            return -1;
        } else {
            std::get<MappedTensorData_u8>(m_tensor_data).data->assign(data, data + num_elements);
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

    virtual bool has_tensor_data() const override
    {
        if (m_shape.empty()) {
            // no shape, no data
            return false;
        }

        // if the shape has no dynamic dimensions, then data is available
        bool has_dynamic_dims = std::any_of(m_shape.begin(), m_shape.end(), [](int64_t dim) { return dim < 0; });
        return !has_dynamic_dims;
    }

    virtual std::string get_dtype_str() const override
    {
        if (m_port_info) {
            return m_port_info->get_dtype_str();
        }
        return "";
    }


    //! Set the port info and shape, the shape should be more specific than the port shape
    //! If shape is provided, it will check if the shape is compatible with the port shape
    //! If not, it will raise an error
    virtual int init(RknnModelPortInfo::Ptr port_info,
                     std::optional<std::vector<int64_t>> shape = std::nullopt)
    {
        m_port_info = port_info;
        if (shape.has_value()) {
            return _set_shape_and_allocate_data(shape.value());
        } else {
            return _set_shape_and_allocate_data(port_info->get_shape());
        }
    }

  protected:
    //! Set the shape
    //! If the shape is not compatible with the port shape, it will raise an error
    //! If the shape is compatible, it will allocate the data
    //! Note that if dynamic dimensions are present in shape, it is considered ok, but data will not be allocated
    virtual int _set_shape_and_allocate_data(const std::vector<int64_t> &shape)
    {
        if (shape == m_shape) {
            // shape is the same, no need to set shape
            // RDX_INFO_DEV(nullptr, __func__, false, "{}", "Shape is the same, no need to set shape");
            return 0;
        }

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
        auto old_shape = m_shape;
        m_shape = shape;

        // allocate data
        {
            bool ok = _allocate_data() == 0;
            if (!ok) {
                RDX_RAISE_ERROR("[f={}] Failed to allocate data", __func__);
                return -1;
            }
        }

        if (on_shape_changed) {
            on_shape_changed(shape, old_shape);
        }

        return 0;
    }
    //! Initialize all internals based on the port info and shape
    //! If you change the shape, you need to call this function again
    virtual int _allocate_data()
    {
        // RDX_INFO_DEV(nullptr, __func__, false, "Allocating data for port: {}", m_port_info->get_name());
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
    RknnModelPortInfo::Ptr m_port_info;
    std::variant<MappedTensorData_f32,
                 MappedTensorData_u8>
        m_tensor_data;
    std::vector<int64_t> m_shape;

  public:
    // callback function to notify shape changed
    // will be called after the shape is changed and data is allocated
    std::function<void(const std::vector<int64_t> &new_shape, const std::vector<int64_t> &old_shape)> on_shape_changed;
};

} // namespace redoxi_works::inference::rknn
