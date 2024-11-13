#include <redoxi_inference_onnx/OnnxModelInference.hpp>
#include <redoxi_inference_onnx/OnnxPortData.hpp>

namespace redoxi_works::inference::onnx
{
    
OnnxModelInference::OnnxModelInference()
{
}

KeyValueStore::Ptr OnnxModelInference::create_key_value_store()
{
    return std::make_shared<OnnxModelConfig>();
}

ModelPortData::Ptr OnnxModelInference::create_model_port_data(
    const std::string& port_name, 
    std::optional<int> batch_size)
{
    // find the port info, if not found, return nullptr
    auto it = m_port_infos.find(port_name);
    if (it == m_port_infos.end()) {
        return nullptr;
    }

    // create the port data
    auto output = std::make_shared<OnnxPortData>();
    
    // if batch size is provided, update the port info
    if (batch_size.has_value()) {
        auto port_info = it->second;
        auto shape = port_info->get_shape();
        if (!shape.empty() && shape[0] == -1) {
            auto new_port_info = std::make_shared<ModelPortInfo>(*port_info);
            new_port_info->get_shape()[0] = batch_size.value();
            output->m_port_info = new_port_info;
        } else {
            // if the port info is not dynamic, check if it matches the batch_size
            if (shape[0] == batch_size.value()) {
                // ok, use the original port info
                output->m_port_info = port_info;
            } else {
                // the batch size does not match, regard it as failure
                return nullptr;
            }
        }
    } else {
        // if batch size is not provided, use the original port info
        output->m_port_info = it->second;
    }
    
    return output;
}

KeyValueStore::ConstPtr OnnxModelInference::get_model_metadata() const
{
    return m_config;
}

KeyValueStore::ConstPtr OnnxModelInference::get_inference_metadata() const
{
    return m_config;
}

ModelPortInfo::ConstPtrMap OnnxModelInference::get_port_infos() const
{
    ModelPortInfo::ConstPtrMap output;
    for (const auto& [name, info] : m_port_infos) {
        output[name] = info;
    }
    return output;
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

}  // namespace redoxi_works::inference::onnx
