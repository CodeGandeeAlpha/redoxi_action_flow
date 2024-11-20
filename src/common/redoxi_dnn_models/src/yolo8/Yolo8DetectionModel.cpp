#include <redoxi_dnn_models/yolo8/Yolo8DetectionModel.hpp>
#include <pluginlib/class_loader.hpp>
namespace redoxi_works::inference::yolo8
{

struct Yolo8DetectionModel::Impl {
    Impl()
        : loader("redoxi_inference",
                 "redoxi_works::inference::RedoxiModelInference")
    {
    }

    // keep the loader alive during the lifetime of the class
    pluginlib::ClassLoader<RedoxiModelInference> loader;

    inline static const std::string PreprocessInfoKey = "preprocess_info";
};

Yolo8DetectionModel::Yolo8DetectionModel()
    : m_impl(std::make_shared<Impl>())
{
}

KeyValueStore::Ptr Yolo8DetectionModel::create_init_params()
{
    return std::make_shared<InitConfig_t>();
}

InferenceInOutData::Ptr Yolo8DetectionModel::create_inference_inout_data()
{
    if (m_model) {
        return m_model->create_inference_inout_data();
    }
    return nullptr;
}

ModelPortInfo::ConstPtrMap Yolo8DetectionModel::get_input_port_infos() const
{
    if (m_model) {
        return m_model->get_input_port_infos();
    }
    return {};
}

ModelPortInfo::ConstPtrMap Yolo8DetectionModel::get_output_port_infos() const
{
    if (m_model) {
        return m_model->get_output_port_infos();
    }
    return {};
}


} // namespace redoxi_works::inference::yolo8
