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

// ModelPortData::Ptr OnnxModelInference::create_model_port_data(
//     const std::string& port_name, 
//     std::optional<int> batch_size)
// {
//     // find the port info, if not found, return nullptr
//     auto it = m_port_infos.find(port_name);
//     if (it == m_port_infos.end()) {
//         return nullptr;
//     }

//     // create the port data
//     auto output = std::make_shared<OnnxPortData>();
    
//     // if batch size is provided, update the port info
//     if (batch_size.has_value()) {
//         auto port_info = it->second;
//         auto shape = port_info->get_shape();
//         if (!shape.empty() && shape[0] == -1) {
//             auto new_port_info = std::make_shared<ModelPortInfo>(*port_info);
//             new_port_info->get_shape()[0] = batch_size.value();
//             output->m_port_info = new_port_info;
//         } else {
//             // if the port info is not dynamic, check if it matches the batch_size
//             if (shape[0] == batch_size.value()) {
//                 // ok, use the original port info
//                 output->m_port_info = port_info;
//             } else {
//                 // the batch size does not match, regard it as failure
//                 return nullptr;
//             }
//         }
//     } else {
//         // if batch size is not provided, use the original port info
//         output->m_port_info = it->second;
//     }
    
//     return output;
// }

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

}  // namespace redoxi_works::inference::onnx
