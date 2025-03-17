#include <redoxi_inference_onnx/OnnxInferenceInOutData.hpp>
#include <redoxi_inference_onnx/OnnxModelInference.hpp>

namespace redoxi_works::inference::onnx
{

RedoxiModelInference *OnnxInferenceInOutData::get_owner()
{
    return m_model_inference;
}

const RedoxiModelInference *OnnxInferenceInOutData::get_owner() const
{
    return m_model_inference;
}

bool OnnxInferenceInOutData::_update_io_binding_input()
{
    if (!m_use_io_binding) {
        // do nothing if io binding is not used
        return false;
    }

    // intended to use io binding, create it if not exist
    if (!m_io_binding) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "IO binding not found, creating io binding");
        // create io binding
        auto session = dynamic_cast<OnnxModelInference *>(m_model_inference)->get_onnx_session();
        m_io_binding = std::make_unique<Ort::IoBinding>(*session);
    }

    // bind all inputs
    RDX_INFO_DEV(nullptr, __func__, "{}", "Binding all input ports");
    for (auto &[port_name, port_data] : m_input_ports) {
        auto dtype_str = port_data->get_dtype_str();
        if (dtype_str == "float32") {
            RDX_INFO_DEV(nullptr, __func__, "Binding input port: {} (float32)", port_name);
            auto &tensor_data_f32 = std::get<MappedTensorData_f32>(port_data->m_tensor_data);

            if (!tensor_data_f32.has_data()) {
                RDX_RAISE_ERROR("Input port {} does not have fixed shape data, failed to bind", port_name);
            }

            auto ort_value = Ort::Value::CreateTensor<float>(
                *tensor_data_f32.onnx_memory_info, tensor_data_f32.data->data(), tensor_data_f32.data->size(),
                tensor_data_f32.shape.data(), tensor_data_f32.shape.size());
            m_io_binding->BindInput(port_name.c_str(), std::move(ort_value));

        } else if (dtype_str == "uint8") {
            RDX_INFO_DEV(nullptr, __func__, "Binding input port: {} (uint8)", port_name);
            auto &tensor_data_u8 = std::get<MappedTensorData_u8>(port_data->m_tensor_data);
            if (!tensor_data_u8.has_data()) {
                RDX_RAISE_ERROR("Input port {} does not have fixed shape data, failed to bind", port_name);
            }

            auto ort_value = Ort::Value::CreateTensor<uint8_t>(
                *tensor_data_u8.onnx_memory_info, tensor_data_u8.data->data(), tensor_data_u8.data->size(),
                tensor_data_u8.shape.data(), tensor_data_u8.shape.size());
            m_io_binding->BindInput(port_name.c_str(), std::move(ort_value));
        } else {
            RDX_RAISE_ERROR("Unsupported data type: {}", dtype_str);
        }
    }

    return true;
}

bool OnnxInferenceInOutData::_update_io_binding_output()
{
    if (!m_use_io_binding) {
        // do nothing if io binding is not used
        return false;
    }

    // intended to use io binding, create it if not exist
    if (!m_io_binding) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "IO binding not found, creating io binding");
        // create io binding
        auto session = dynamic_cast<OnnxModelInference *>(m_model_inference)->get_onnx_session();
        m_io_binding = std::make_unique<Ort::IoBinding>(*session);
    }

    // did we already bind all outputs?
    bool need_to_update_binding = false;
    bool reset_dirty_flag = false;
    {
        RDX_INFO_DEV(nullptr, __func__,
                     "Checking if all output ports are bound, preferred bind to tensor = {}", m_prefer_bind_output_tensor);
        auto bound_output_port_names = m_io_binding->GetOutputNames();
        if (bound_output_port_names.size() != m_output_ports.size()) {
            // some output ports are not bound yet
            need_to_update_binding = true;
        }

        // all bound, do we have shape changed?
        if (m_prefer_bind_output_tensor && m_output_port_configuration_dirty) {
            need_to_update_binding = true;

            // we will bind it now, after we deal with this, reset the flag
            reset_dirty_flag = true;
        }
    }

    if (!need_to_update_binding) {
        // no need to update binding
        RDX_INFO_DEV(nullptr, __func__, "{}", "No need to update binding, they are either already bound or shape does not changed");
        return false;
    }

    // bind all outputs, if it has fixed shape and bind-to-tensor is requested, bind the tensor directly,
    // otherwise bind the memory info
    m_output_port_bound_by_tensor.clear();
    RDX_INFO_DEV(nullptr, __func__, "{}", "Binding all output ports");
    for (auto &[port_name, port_data] : m_output_ports) {
        RDX_INFO_DEV(nullptr, __func__, "Binding output port: {}", port_name);
        auto dtype_str = port_data->get_dtype_str();
        if (dtype_str == "float32") {
            auto &tensor_data_f32 = std::get<MappedTensorData_f32>(port_data->m_tensor_data);
            if (tensor_data_f32.has_data() && m_prefer_bind_output_tensor) {
                RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (float32) with tensor", port_name);
                auto ort_value = Ort::Value::CreateTensor<float>(
                    *tensor_data_f32.onnx_memory_info, tensor_data_f32.data->data(), tensor_data_f32.data->size(),
                    tensor_data_f32.shape.data(), tensor_data_f32.shape.size());
                m_io_binding->BindOutput(port_name.c_str(), std::move(ort_value));
                m_output_port_bound_by_tensor[port_name] = true;
            } else {
                if (m_prefer_bind_output_tensor) {
                    RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (float32) with memory info, because tensor data is not available", port_name);
                } else {
                    RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (float32) with memory info as requested", port_name);
                }
                m_io_binding->BindOutput(port_name.c_str(), *tensor_data_f32.onnx_memory_info);
                m_output_port_bound_by_tensor[port_name] = false;
            }
        } else if (dtype_str == "uint8") {
            auto &tensor_data_u8 = std::get<MappedTensorData_u8>(port_data->m_tensor_data);
            if (tensor_data_u8.has_data() && m_prefer_bind_output_tensor) {
                RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (uint8) with tensor", port_name);
                auto ort_value = Ort::Value::CreateTensor<uint8_t>(
                    *tensor_data_u8.onnx_memory_info, tensor_data_u8.data->data(), tensor_data_u8.data->size(),
                    tensor_data_u8.shape.data(), tensor_data_u8.shape.size());
                m_io_binding->BindOutput(port_name.c_str(), std::move(ort_value));
                m_output_port_bound_by_tensor[port_name] = true;
            } else {
                if (m_prefer_bind_output_tensor) {
                    RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (uint8) with memory info, because tensor data is not available", port_name);
                } else {
                    RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (uint8) with memory info as requested", port_name);
                }
                m_io_binding->BindOutput(port_name.c_str(), *tensor_data_u8.onnx_memory_info);
                m_output_port_bound_by_tensor[port_name] = false;
            }
        } else {
            RDX_RAISE_ERROR("Unsupported data type: {}", dtype_str);
        }
    }

    // reset the dirty flag if needed
    if (reset_dirty_flag) {
        m_output_port_configuration_dirty = false;
    }

    return true;
}

void OnnxInferenceInOutData::init(OnnxModelInference *model_inference)
{
    m_model_inference = model_inference;
    m_io_binding.reset();
    m_input_ports.clear();
    m_output_ports.clear();

    // initialize all ports
    auto input_port_infos = model_inference->_get_input_port_infos();
    for (auto &[port_name, port_info] : input_port_infos) {
        auto port_data = std::make_shared<OnnxPortData>();
        auto _port_info = std::dynamic_pointer_cast<OnnxModelPortInfo>(port_info);
        if (!_port_info) {
            RDX_RAISE_ERROR("port info type is not OnnxModelPortInfo: {}", port_name);
        }
        port_data->init(_port_info);
        m_input_ports[port_name] = port_data;

        // add callback function to notify when the shape is changed
        port_data->on_shape_changed = [this](const std::vector<int64_t> &, const std::vector<int64_t> &) {
            m_input_port_configuration_dirty = true;
        };
    }

    auto output_port_infos = model_inference->_get_output_port_infos();
    for (auto &[port_name, port_info] : output_port_infos) {
        auto port_data = std::make_shared<OnnxPortData>();
        auto _port_info = std::dynamic_pointer_cast<OnnxModelPortInfo>(port_info);
        if (!_port_info) {
            RDX_RAISE_ERROR("port info type is not OnnxModelPortInfo: {}", port_name);
        }
        port_data->init(_port_info);
        m_output_ports[port_name] = port_data;

        // add callback function to notify when the shape is changed
        port_data->on_shape_changed = [this](const std::vector<int64_t> &, const std::vector<int64_t> &) {
            m_output_port_configuration_dirty = true;
        };
    }

    // initially, all ports are dirty because they are just created now
    m_input_port_configuration_dirty = true;
    m_output_port_configuration_dirty = true;
}

ModelPortData::Ptr OnnxInferenceInOutData::get_input_port_data(const std::string &port_name)
{
    auto it = m_input_ports.find(port_name);
    if (it != m_input_ports.end()) {
        return it->second;
    }
    return nullptr;
}

ModelPortData::Ptr OnnxInferenceInOutData::get_output_port_data(const std::string &port_name)
{
    auto it = m_output_ports.find(port_name);
    if (it != m_output_ports.end()) {
        return it->second;
    }
    return nullptr;
}

ModelPortInfo::ConstPtr OnnxInferenceInOutData::get_input_port_info(const std::string &port_name) const
{
    auto it = m_input_ports.find(port_name);
    if (it != m_input_ports.end()) {
        return it->second->get_port_info();
    }
    return nullptr;
}

ModelPortInfo::ConstPtr OnnxInferenceInOutData::get_output_port_info(const std::string &port_name) const
{
    auto it = m_output_ports.find(port_name);
    if (it != m_output_ports.end()) {
        return it->second->get_port_info();
    }
    return nullptr;
}

std::shared_ptr<std::any> OnnxInferenceInOutData::get_any_data(const std::string &key) const
{
    auto it = m_any_data.find(key);
    if (it != m_any_data.end()) {
        return it->second;
    }
    return nullptr;
}

std::map<std::string, std::shared_ptr<std::any>> OnnxInferenceInOutData::get_any_data() const
{
    return m_any_data;
}

void OnnxInferenceInOutData::set_any_data(const std::string &key, std::shared_ptr<std::any> value)
{
    m_any_data[key] = value;
}

bool OnnxInferenceInOutData::remove_any_data(const std::string &key)
{
    auto it = m_any_data.find(key);
    if (it != m_any_data.end()) {
        m_any_data.erase(it);
        return true;
    }
    return false;
}

} // namespace redoxi_works::inference::onnx
