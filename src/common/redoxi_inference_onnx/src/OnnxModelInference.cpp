#include <redoxi_inference_onnx/OnnxModelInference.hpp>
#include <redoxi_inference_onnx/OnnxPortData.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

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

int OnnxModelInference::init(KeyValueStore::Ptr params)
{
    auto config = std::dynamic_pointer_cast<OnnxModelConfig>(params);
    if (config == nullptr) {
        // wrong type of params
        return -1;
    }


    m_config = config;
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
    const std::shared_ptr<Ort::Session> &session)
{
    OnnxModelPortInfo::PtrMap ports;
    size_t num_input_nodes = session->GetInputCount();

    for (size_t i = 0; i < num_input_nodes; ++i) {
        auto input_name = session->GetInputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        auto input_type_info = session->GetInputTypeInfo(i);
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
    const std::shared_ptr<Ort::Session> &session)
{
    OnnxModelPortInfo::PtrMap ports;
    size_t num_output_nodes = session->GetOutputCount();

    for (size_t i = 0; i < num_output_nodes; ++i) {
        auto output_name = session->GetOutputNameAllocated(i, Ort::AllocatorWithDefaultOptions());
        auto output_type_info = session->GetOutputTypeInfo(i);
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
            return "float";
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
            return "double";
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
