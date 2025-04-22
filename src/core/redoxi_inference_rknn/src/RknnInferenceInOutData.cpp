#include <redoxi_inference_rknn/RknnInferenceInOutData.hpp>
#include <redoxi_inference_rknn/RknnModelInference.hpp>

namespace redoxi_works::inference::rknn
{

RedoxiModelInference *RknnInferenceInOutData::get_owner()
{
    return m_model_inference;
}

const RedoxiModelInference *RknnInferenceInOutData::get_owner() const
{
    return m_model_inference;
}

void RknnInferenceInOutData::init(RknnModelInference *model_inference)
{
    m_model_inference = model_inference;
    m_input_ports.clear();
    m_output_ports.clear();

    // initialize all ports
    auto input_port_infos = model_inference->_get_input_port_infos();
    for (auto &[port_name, port_info] : input_port_infos) {
        auto port_data = std::make_shared<RknnPortData>();
        auto _port_info = std::dynamic_pointer_cast<RknnModelPortInfo>(port_info);
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
        auto port_data = std::make_shared<RknnPortData>();
        auto _port_info = std::dynamic_pointer_cast<RknnModelPortInfo>(port_info);
        if (!_port_info) {
            RDX_RAISE_ERROR("port info type is not RknnModelPortInfo: {}", port_name);
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

ModelPortData::Ptr RknnInferenceInOutData::get_input_port_data(const std::string &port_name)
{
    auto it = m_input_ports.find(port_name);
    if (it != m_input_ports.end()) {
        return it->second;
    }
    return nullptr;
}

ModelPortData::Ptr RknnInferenceInOutData::get_output_port_data(const std::string &port_name)
{
    auto it = m_output_ports.find(port_name);
    if (it != m_output_ports.end()) {
        return it->second;
    }
    return nullptr;
}

ModelPortInfo::ConstPtr RknnInferenceInOutData::get_input_port_info(const std::string &port_name) const
{
    auto it = m_input_ports.find(port_name);
    if (it != m_input_ports.end()) {
        return it->second->get_port_info();
    }
    return nullptr;
}

ModelPortInfo::ConstPtr RknnInferenceInOutData::get_output_port_info(const std::string &port_name) const
{
    auto it = m_output_ports.find(port_name);
    if (it != m_output_ports.end()) {
        return it->second->get_port_info();
    }
    return nullptr;
}

std::shared_ptr<std::any> RknnInferenceInOutData::get_any_data(const std::string &key) const
{
    auto it = m_any_data.find(key);
    if (it != m_any_data.end()) {
        return it->second;
    }
    return nullptr;
}

std::map<std::string, std::shared_ptr<std::any>> RknnInferenceInOutData::get_any_data() const
{
    return m_any_data;
}

void RknnInferenceInOutData::set_any_data(const std::string &key, std::shared_ptr<std::any> value)
{
    m_any_data[key] = value;
}

bool RknnInferenceInOutData::remove_any_data(const std::string &key)
{
    auto it = m_any_data.find(key);
    if (it != m_any_data.end()) {
        m_any_data.erase(it);
        return true;
    }
    return false;
}

} // namespace redoxi_works::inference::rknn
