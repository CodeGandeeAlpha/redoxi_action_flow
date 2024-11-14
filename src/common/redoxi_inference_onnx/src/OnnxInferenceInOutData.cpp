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

void OnnxInferenceInOutData::notify_input_data_update()
{
    if (m_io_binding) {
        m_io_binding->SynchronizeInputs();
    }
}

void OnnxInferenceInOutData::notify_input_configure_update()
{
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

    // initialize all ports
    auto input_port_infos = model_inference->get_input_port_infos();
    for (auto &[port_name, port_info] : input_port_infos) {
        auto port_data = std::make_shared<OnnxPortData>();
        auto _port_info = std::dynamic_pointer_cast<OnnxModelPortInfo>(port_info);
        if (!_port_info) {
            RDX_RAISE_ERROR("port info type is not OnnxModelPortInfo: {}", port_name);
        }
        port_data->init(_port_info);
        m_input_ports[port_name] = port_data;
    }

    auto output_port_infos = model_inference->get_output_port_infos();
    for (auto &[port_name, port_info] : output_port_infos) {
        auto port_data = std::make_shared<OnnxPortData>();
        auto _port_info = std::dynamic_pointer_cast<OnnxModelPortInfo>(port_info);
        if (!_port_info) {
            RDX_RAISE_ERROR("port info type is not OnnxModelPortInfo: {}", port_name);
        }
        port_data->init(_port_info);
        m_output_ports[port_name] = port_data;
    }

    // notify the model inference to update the io binding, if possible
    notify_input_configure_update();
}

} // namespace redoxi_works::inference::onnx
