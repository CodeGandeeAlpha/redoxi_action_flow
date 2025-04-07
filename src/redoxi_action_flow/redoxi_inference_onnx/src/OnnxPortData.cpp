#include <redoxi_inference_onnx/OnnxPortData.hpp>
#include <redoxi_basic_cpp/logging/ros_logging.hpp>

namespace redoxi_works::inference::onnx
{
int OnnxPortData::set_tensor_data(const float *data, std::vector<int64_t> shape,
                                  std::optional<TensorFormat> fmt)
{
    // check dtype compatibility first
    if (m_port_info->get_dtype_str() != "float32") {
        // dtype mismatch
        RDX_RAISE_ERROR("[f={}()] data type mismatch, expected float32, got {}", __func__,
                        m_port_info->get_dtype_str());
    }

    // TODO: currently only support NCHW format
    if (fmt.has_value() && fmt.value() != TensorFormat::NCHW) {
        // fmt mismatch
        RDX_RAISE_ERROR("[f={}()] currently only support NCHW format", __func__);
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
        return 0;
    }
}

int OnnxPortData::set_tensor_data(const uint8_t *data, std::vector<int64_t> shape,
                                  std::optional<TensorFormat> fmt)
{
    // check dtype compatibility first
    if (m_port_info->get_dtype_str() != "uint8") {
        // dtype mismatch
        RDX_RAISE_ERROR("[f={}()] data type mismatch, expected uint8, got {}", __func__,
                        m_port_info->get_dtype_str());
    }

    // TODO: currently only support NCHW format
    if (fmt.has_value() && fmt.value() != TensorFormat::NCHW) {
        // fmt mismatch
        RDX_RAISE_ERROR("[f={}()] currently only support NCHW format", __func__);
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

ModelPortInfo::ConstPtr OnnxPortData::get_port_info() const
{
    return m_port_info;
}

int OnnxPortData::get_tensor_data(const float **output_tensor) const
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

int OnnxPortData::get_tensor_data(float **output_tensor)
{
    return get_tensor_data(const_cast<const float **>(output_tensor));
}

int OnnxPortData::get_tensor_data(const uint8_t **output_tensor) const
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

int OnnxPortData::get_tensor_data(uint8_t **output_tensor)
{
    return get_tensor_data(const_cast<const uint8_t **>(output_tensor));
}

std::vector<int64_t> OnnxPortData::get_shape() const
{
    return m_shape;
}

bool OnnxPortData::has_tensor_data() const
{
    if (m_shape.empty()) {
        // no shape, no data
        return false;
    }

    // if the shape has no dynamic dimensions, then data is available
    bool has_dynamic_dims = std::any_of(m_shape.begin(), m_shape.end(), [](int64_t dim) { return dim < 0; });
    return !has_dynamic_dims;
}

std::string OnnxPortData::get_dtype_str() const
{
    if (m_port_info) {
        return m_port_info->get_dtype_str();
    }
    return "";
}

int OnnxPortData::init(OnnxModelPortInfo::Ptr port_info, std::optional<std::vector<int64_t>> shape)
{
    m_port_info = port_info;
    if (shape.has_value()) {
        return _set_shape_and_allocate_data(shape.value());
    } else {
        return _set_shape_and_allocate_data(port_info->get_shape());
    }
}

int OnnxPortData::_set_shape_and_allocate_data(const std::vector<int64_t> &shape)
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

int OnnxPortData::_allocate_data()
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

} // namespace redoxi_works::inference::onnx
