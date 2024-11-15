#include <redoxi_inference_onnx/OnnxModelInference.hpp>
#include <redoxi_inference_onnx/OnnxPortData.hpp>
#include <redoxi_inference_onnx/OnnxInferenceInOutData.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <filesystem>

namespace redoxi_works::inference::onnx
{

OnnxModelInference::OnnxModelInference()
{
}

KeyValueStore::Ptr OnnxModelInference::create_init_params()
{
    return std::make_shared<OnnxModelConfig>();
}

KeyValueStore::ConstPtr OnnxModelInference::get_model_metadata() const
{
    return m_config;
}

KeyValueStore::ConstPtr OnnxModelInference::get_inference_metadata() const
{
    return m_config;
}

ModelPortInfo::ConstPtrMap OnnxModelInference::get_input_port_infos() const
{
    ModelPortInfo::ConstPtrMap ret;
    for (const auto &[port_name, port_info] : m_input_ports) {
        ret[port_name] = port_info;
    }
    return ret;
}

ModelPortInfo::ConstPtrMap OnnxModelInference::get_output_port_infos() const
{
    ModelPortInfo::ConstPtrMap ret;
    for (const auto &[port_name, port_info] : m_output_ports) {
        ret[port_name] = port_info;
    }
    return ret;
}

InferenceInOutData::Ptr OnnxModelInference::create_inference_inout_data()
{
    auto output = std::make_shared<OnnxInferenceInOutData>();

    RDX_INFO_DEV(nullptr, __func__, false, "{}", "Initializing inference inout data");
    output->init(this);
    return output;
}

int OnnxModelInference::do_inference(InferenceInOutData::Ptr inout_data)
{
    RDX_INFO_DEV(nullptr, __func__, false, "{}", "Performing inference");

    // do we have io binding?
    auto onnx_inout_data = std::dynamic_pointer_cast<OnnxInferenceInOutData>(inout_data);
    auto io_binding = onnx_inout_data->m_io_binding;
    if (io_binding != nullptr) {
        // has io binding, notify the inout data to update the io binding
        RDX_INFO_DEV(nullptr, __func__, false, "{}", "IO binding found, performing inference");

        // perform inference
        io_binding->SynchronizeInputs();
        m_session->Run(Ort::RunOptions{nullptr}, *io_binding);
        io_binding->SynchronizeOutputs();
    } else {
        RDX_INFO_DEV(nullptr, __func__, false, "{}", "No IO binding found, performing inference without IO binding");

        // get all input data
        std::vector<Ort::Value> ort_values;
        std::vector<const char *> input_names;
        for (const auto &[port_name, port_data] : onnx_inout_data->m_input_ports) {
            float *tdata = nullptr;
            port_data->get_tensor_data(&tdata);
            auto shape = port_data->get_shape();
            input_names.push_back(port_name.c_str());

            // get all ort values
            if (std::holds_alternative<MappedTensorData_f32>(port_data->m_tensor_data)) {
                auto v = std::get<MappedTensorData_f32>(port_data->m_tensor_data).onnx_tensor;
                ort_values.push_back(*v);
            } else if (std::holds_alternative<MappedTensorData_u8>(port_data->m_tensor_data)) {
                auto v = std::get<MappedTensorData_u8>(port_data->m_tensor_data).onnx_tensor;
                ort_values.push_back(*v);
            } else {
                RDX_RAISE_ERROR("[f={}] Cannot find suitable onnx tensor for port: {}", __func__, port_name);
                return -1;
            }
        }

        // get all output names
        std::vector<const char *> output_names;
        for (const auto &[port_name, port_data] : onnx_inout_data->m_output_ports) {
            output_names.push_back(port_name.c_str());
        }

        if (output_names.empty()) {
            RDX_RAISE_ERROR("[f={}] No output names found", __func__);
            return -1;
        }

        // perform inference
        RDX_INFO_DEV(nullptr, __func__, false, "{}", "Performing inference");
        auto ort_outputs = m_session->Run(Ort::RunOptions{nullptr},
                                          input_names.data(), ort_values.data(), ort_values.size(),
                                          output_names.data(), output_names.size());

        // write output data to ports
        RDX_INFO_DEV(nullptr, __func__, false, "{}", "Writing output data to ports");
        for (size_t i = 0; i < ort_outputs.size(); ++i) {
            auto port_name = output_names[i];
            auto port_data = onnx_inout_data->m_output_ports[port_name];
            auto &ov = ort_outputs[i];
            auto o_shape = ov.GetTensorTypeAndShapeInfo().GetShape();
            auto o_dtype = ov.GetTensorTypeAndShapeInfo().GetElementType();

            RDX_INFO_DEV(nullptr, __func__, false,
                         "Writing to output port: {}, shape: {}, dtype: {}",
                         port_name, fmt::join(o_shape, ","),
                         onnx_element_type_to_string(o_dtype));

            if (o_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
                float *raw_output_data = ov.GetTensorMutableData<float>();
                port_data->set_tensor_data(raw_output_data, o_shape);
            } else if (o_dtype == ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8) {
                uint8_t *raw_output_data = ov.GetTensorMutableData<uint8_t>();
                port_data->set_tensor_data(raw_output_data, o_shape);
            } else {
                RDX_RAISE_ERROR("[f={}] Unsupported output data type: {}", __func__, o_dtype);
                return -1;
            }
        }
    }

    return 0;
}

int OnnxModelInference::open(KeyValueStore::Ptr params)
{
    auto config = std::dynamic_pointer_cast<OnnxModelConfig>(params);
    if (config == nullptr) {
        // wrong type of params
        return -1;
    }

    auto model_path = config->model_path;
    auto provider_type = config->execution_provider;
    auto logging_level = config->logging_level;

    // check if the model path is valid
    if (model_path.empty()) {
        RDX_RAISE_ERROR("[f={}] Model path is empty", __func__);
        return -1;
    }
    if (!std::filesystem::exists(model_path)) {
        RDX_RAISE_ERROR("[f={}] Model path does not exist: {}", __func__, model_path);
        return -1;
    }
    if (provider_type.empty()) {
        RDX_RAISE_ERROR("[f={}] Provider type is empty", __func__);
        return -1;
    }

    // create the environment and session
    m_env = std::make_shared<Ort::Env>(logging_level, config->log_id.c_str());
    m_session = create_onnx_session(model_path, provider_type, *m_env);

    // get all input and output port infos
    m_input_ports = get_input_port_infos(*m_session);
    RDX_INFO_DEV(nullptr, __func__, false, "Got {} input ports", m_input_ports.size());
    {
        for (const auto &[port_name, port_info] : m_input_ports) {
            RDX_INFO_DEV(nullptr, __func__, false, "Input Port: {}", port_info->to_description());
        }
    }

    m_output_ports = get_output_port_infos(*m_session);
    RDX_INFO_DEV(nullptr, __func__, false, "Got {} output ports", m_output_ports.size());
    {
        for (const auto &[port_name, port_info] : m_output_ports) {
            RDX_INFO_DEV(nullptr, __func__, false, "Output Port: {}", port_info->to_description());
        }
    }

    RDX_INFO_DEV(nullptr, __func__, false, "{}", "Initialization completed");
    m_config = config;
    return 0;
}

bool OnnxModelInference::is_open() const
{
    return m_session != nullptr;
}

int OnnxModelInference::close()
{
    // close session, unload model
    m_session = nullptr;
    m_env = nullptr;
    return 0;
}

std::shared_ptr<Ort::Session> OnnxModelInference::create_onnx_session(
    const std::string &model_path,
    const std::string &provider_type,
    Ort::Env &env)
{
    //! Create an ONNX session and load the model based on the provider type
    Ort::SessionOptions session_options;
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    RDX_INFO_DEV(nullptr, __func__, false, "Configuring execution providier {}", provider_type);
    if (provider_type == onnx_ep_names::CUDA) {
        OrtCUDAProviderOptions cuda_options;
        session_options.AppendExecutionProvider_CUDA(cuda_options);
    } else if (provider_type == onnx_ep_names::CPU) {
        // No additional configuration needed for CPU
    } else if (provider_type == onnx_ep_names::TensorRT) {
        RDX_INFO_DEV(nullptr, __func__, false, "Configuring execution provider {}", provider_type);
        OrtTensorRTProviderOptions trt_options;
        trt_options.device_id = 0;
        trt_options.trt_max_partition_iterations = 10;
        session_options.AppendExecutionProvider_TensorRT(trt_options);
    } else {
        RDX_LOG_ERROR(nullptr, __func__, false, "Unknown execution provider: {}", provider_type);
        return nullptr;
    }

    std::shared_ptr<Ort::Session> session;
    try {
        RDX_INFO_DEV(nullptr, __func__, false, "Creating session with model path: {}", model_path);
        session = std::make_shared<Ort::Session>(env, model_path.c_str(), session_options);
    } catch (const Ort::Exception &e) {
        RDX_LOG_ERROR(nullptr, __func__, false, "Error creating session: {}", e.what());
        return nullptr;
    }

    return session;
}

OnnxModelPortInfo::PtrMap OnnxModelInference::get_input_port_infos(
    const Ort::Session &session)
{
    OnnxModelPortInfo::PtrMap ports;
    size_t num_input_nodes = session.GetInputCount();

    for (size_t i = 0; i < num_input_nodes; ++i) {
        auto input_name = session.GetInputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        auto input_type_info = session.GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();

        auto port_info = std::make_shared<OnnxModelPortInfo>();
        port_info->m_name = input_name.get();
        port_info->m_is_input = true;
        port_info->m_index = i;
        port_info->m_dtype = input_tensor_info.GetElementType();
        port_info->m_dtype_str = onnx_element_type_to_string(port_info->m_dtype);
        port_info->m_shape = input_tensor_info.GetShape();

        ports[port_info->m_name] = port_info;
    }

    return ports;
}

OnnxModelPortInfo::PtrMap OnnxModelInference::get_output_port_infos(
    const Ort::Session &session)
{
    OnnxModelPortInfo::PtrMap ports;
    size_t num_output_nodes = session.GetOutputCount();

    for (size_t i = 0; i < num_output_nodes; ++i) {
        auto output_name = session.GetOutputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        auto output_type_info = session.GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();

        auto port_info = std::make_shared<OnnxModelPortInfo>();
        port_info->m_name = output_name.get();
        port_info->m_is_input = false;
        port_info->m_index = i;
        port_info->m_dtype = output_tensor_info.GetElementType();
        port_info->m_dtype_str = onnx_element_type_to_string(port_info->m_dtype);
        port_info->m_shape = output_tensor_info.GetShape();

        ports[port_info->m_name] = port_info;
    }

    return ports;
}

std::string OnnxModelInference::onnx_element_type_to_string(ONNXTensorElementDataType dtype)
{
    switch (dtype) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
            return "float32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
            return "uint8";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
            return "int8";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
            return "uint16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
            return "int16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
            return "int32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64:
            return "int64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_STRING:
            return "string";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL:
            return "bool";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16:
            return "float16";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_DOUBLE:
            return "float64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT32:
            return "uint32";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT64:
            return "uint64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX64:
            return "complex64";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_COMPLEX128:
            return "complex128";
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BFLOAT16:
            return "bfloat16";
        default:
            return "unknown";
    }
}

} // namespace redoxi_works::inference::onnx
