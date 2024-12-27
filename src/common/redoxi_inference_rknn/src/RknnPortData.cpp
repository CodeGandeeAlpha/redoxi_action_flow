#include <redoxi_inference_rknn/RknnPortData.hpp>
#include <redoxi_basic_cpp/logging/ros_logging.hpp>
#include <numeric>
#include <algorithm>

namespace redoxi_works::inference::rknn
{

ModelPortInfo::ConstPtr RknnPortData::get_port_info() const
{
    return m_port_info;
}

int RknnPortData::set_tensor_data(const float *data, std::vector<int64_t> shape, std::optional<TensorFormat> format)
{
    //! Check dtype compatibility first
    if (m_port_info->get_dtype_str() != "float32") {
        return -1;
    }

    // check if the format is compatible with the port info
    if (format.has_value() && format.value() != m_port_info->get_tensor_format()) {
        RDX_RAISE_ERROR("[f={}] Tensor format mismatch: {} vs {}",
                        __func__, tensor_format_to_string(format.value()),
                        tensor_format_to_string(m_port_info->get_tensor_format()));
        return -1;
    }

    //! Set shape and allocate data
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
        return 0;
    }
}

int RknnPortData::set_tensor_data(const uint8_t *data, std::vector<int64_t> shape, std::optional<TensorFormat> format)
{
    //! Check dtype compatibility first
    if (m_port_info->get_dtype_str() != "uint8") {
        return -1;
    }

    //! Check if the format is compatible with the port info
    if (format.has_value() && format.value() != m_port_info->get_tensor_format()) {
        RDX_RAISE_ERROR("[f={}] Tensor format mismatch: {} vs {}",
                        __func__, tensor_format_to_string(format.value()),
                        tensor_format_to_string(m_port_info->get_tensor_format()));
        return -1;
    }

    //! Set shape and allocate data
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

int RknnPortData::get_tensor_data(const float **output_tensor) const
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

int RknnPortData::get_tensor_data(float **output_tensor)
{
    return get_tensor_data(const_cast<const float **>(output_tensor));
}

int RknnPortData::get_tensor_data(const uint8_t **output_tensor) const
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

int RknnPortData::get_tensor_data(uint8_t **output_tensor)
{
    return get_tensor_data(const_cast<const uint8_t **>(output_tensor));
}

std::vector<int64_t> RknnPortData::get_shape() const
{
    return m_shape;
}

bool RknnPortData::has_tensor_data() const
{
    if (m_shape.empty()) {
        //! No shape, no data
        return false;
    }

    //! If the shape has no dynamic dimensions, then data is available
    bool has_dynamic_dims = std::any_of(m_shape.begin(), m_shape.end(), [](int64_t dim) { return dim < 0; });
    return !has_dynamic_dims;
}

std::string RknnPortData::get_dtype_str() const
{
    if (m_port_info) {
        return m_port_info->get_dtype_str();
    }
    return "";
}

int RknnPortData::init(RknnModelPortInfo::Ptr port_info, std::optional<std::vector<int64_t>> shape)
{
    m_port_info = port_info;
    if (shape.has_value()) {
        return _set_shape_and_allocate_data(shape.value());
    } else {
        return _set_shape_and_allocate_data(port_info->get_shape());
    }
}

int RknnPortData::_set_shape_and_allocate_data(const std::vector<int64_t> &shape)
{
    if (shape == m_shape) {
        return 0;
    }

    //! You must have port info to set the shape
    if (!m_port_info) {
        RDX_RAISE_ERROR("[f={}] Port info not set", __func__);
        return -1;
    }

    //! Check if the shape is compatible with the port shape
    const auto &port_shape = m_port_info->get_shape();
    bool ok = is_shape_compatible(shape, port_shape);
    if (!ok) {
        std::string shape_str = fmt::format("{}", fmt::join(shape, ", "));
        std::string port_shape_str = fmt::format("{}", fmt::join(port_shape, ", "));
        RDX_RAISE_ERROR("[f={}] Shape mismatch: {} vs {}", __func__, shape_str, port_shape_str);
        return -1;
    }

    //! Set the shape
    auto old_shape = m_shape;
    m_shape = shape;

    //! Allocate data
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

int RknnPortData::_allocate_data()
{
    //! Do we have port info?
    if (!m_port_info) {
        RDX_RAISE_ERROR("[f={}] Port info not found", __func__);
        return -1;
    }

    //! Do we have shape?
    if (m_shape.empty()) {
        RDX_RAISE_ERROR("[f={}] Shape not set", __func__);
        return -1;
    }

    //! Allocate data
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

} // namespace redoxi_works::inference::rknn