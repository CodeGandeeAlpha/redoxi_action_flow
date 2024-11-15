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

void OnnxInferenceInOutData::_update_port_configuration()
{
    // just do not bind
    if (!m_port_configuration_dirty) {
        // no need to update
        return;
    }

    // FIXME: should be able to explicitly disable io binding
    // otherwise, if shape of input changes frequently, it will be very slow

    // reset the flag, and then proceed to update the port configuration
    m_port_configuration_dirty = false;

    if (!m_io_binding) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "IO binding not found, creating io binding");

        // create io binding
        auto session = dynamic_cast<OnnxModelInference *>(m_model_inference)->get_onnx_session();
        m_io_binding = std::make_unique<Ort::IoBinding>(*session);
    }

    // bind all inputs
    bool ok = true;
    RDX_INFO_DEV(nullptr, __func__, "{}", "Binding all input ports");
    for (auto &[port_name, port_data] : m_input_ports) {
        auto dtype_str = port_data->get_dtype_str();
        if (dtype_str == "float32") {
            RDX_INFO_DEV(nullptr, __func__, "Binding input port: {} (float32)", port_name);
            auto &tensor_data_f32 = std::get<MappedTensorData_f32>(port_data->m_tensor_data);
            if (tensor_data_f32.has_data()) {
                m_io_binding->BindInput(port_name.c_str(), *tensor_data_f32.onnx_tensor);
            } else {
                RDX_INFO_DEV(nullptr, __func__, "Input port {} has no data, skip IO binding", port_name);
                ok = false;
                break;
            }
        } else if (dtype_str == "uint8") {
            RDX_INFO_DEV(nullptr, __func__, "Binding input port: {} (uint8)", port_name);
            auto &tensor_data_u8 = std::get<MappedTensorData_u8>(port_data->m_tensor_data);
            if (tensor_data_u8.has_data()) {
                m_io_binding->BindInput(port_name.c_str(), *tensor_data_u8.onnx_tensor);
            } else {
                RDX_INFO_DEV(nullptr, __func__, "Input port {} has no data, skip IO binding", port_name);
                ok = false;
                break;
            }
        } else {
            RDX_RAISE_ERROR("Unsupported data type: {}", dtype_str);
        }
    }

    // FIXME: do not use output binding for now, it may have problem
    // bind all outputs, if it has fixed shape, bind the tensor directly, otherwise bind the memory info
    RDX_INFO_DEV(nullptr, __func__, "{}", "Binding all output ports");
    for (auto &[port_name, port_data] : m_output_ports) {
        RDX_INFO_DEV(nullptr, __func__, "Binding output port: {}", port_name);
        auto dtype_str = port_data->get_dtype_str();
        if (dtype_str == "float32") {
            auto &tensor_data_f32 = std::get<MappedTensorData_f32>(port_data->m_tensor_data);
            if (tensor_data_f32.has_data()) {
                RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (float32) with tensor", port_name);
                m_io_binding->BindOutput(port_name.c_str(), *tensor_data_f32.onnx_tensor);
                // Ort::Value output_tensor = Ort::Value::CreateTensor<float>(
                //     *tensor_data_f32.onnx_memory_info,
                //     tensor_data_f32.data->data(), tensor_data_f32.data->size(),
                //     tensor_data_f32.shape.data(), tensor_data_f32.shape.size());
                // m_io_binding->BindOutput(port_name.c_str(), std::move(output_tensor));
            } else {
                RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (float32) with memory info", port_name);
                m_io_binding->BindOutput(port_name.c_str(), *tensor_data_f32.onnx_memory_info);
                // Ort::MemoryInfo memory_info("Cuda", OrtAllocatorType::OrtDeviceAllocator, 0, OrtMemType::OrtMemTypeDefault);
                // m_io_binding->BindOutput(port_name.c_str(), memory_info);
            }
        } else if (dtype_str == "uint8") {
            auto &tensor_data_u8 = std::get<MappedTensorData_u8>(port_data->m_tensor_data);
            if (tensor_data_u8.has_data()) {
                RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (uint8) with tensor", port_name);
                m_io_binding->BindOutput(port_name.c_str(), *tensor_data_u8.onnx_tensor);
            } else {
                RDX_INFO_DEV(nullptr, __func__, "Binding output port: {} (uint8) with memory info", port_name);
                m_io_binding->BindOutput(port_name.c_str(), *tensor_data_u8.onnx_memory_info);
            }
        }
    }

    if (!ok) {
        // failed to bind, just delete io binding
        RDX_INFO_DEV(nullptr, __func__, "{}", "Failed to bind input ports, deleting io binding");
        m_io_binding.reset();
    }
}

void OnnxInferenceInOutData::init(OnnxModelInference *model_inference)
{
    m_model_inference = model_inference;
    m_input_ports.clear();
    m_output_ports.clear();

    // let this class know when the port shape is changed
    // so that it can check and bind the ports accordingly
    auto shape_changed_callback = [this](const std::vector<int64_t> &, const std::vector<int64_t> &) {
        m_port_configuration_dirty = true;
    };

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
        port_data->on_shape_changed = shape_changed_callback;
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

        // FIXME: output port's shape should not change frequently, otherwise it will be slow
        // because io binding will be recreated every time when the shape is changed

        // add callback function to notify when the shape is changed
        // port_data->on_shape_changed = shape_changed_callback;
    }

    // set the flag to indicate the input configuration is dirty
    m_port_configuration_dirty = true;
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

} // namespace redoxi_works::inference::onnx
