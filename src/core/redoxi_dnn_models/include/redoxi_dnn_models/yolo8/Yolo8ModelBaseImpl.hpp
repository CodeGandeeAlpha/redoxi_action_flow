#include <redoxi_dnn_models/yolo8/Yolo8ModelBase.hpp>
#include <pluginlib/class_loader.hpp>

namespace redoxi_works::inference::yolo8
{
struct Yolo8ModelBase::Impl {
    Impl()
        : loader("redoxi_inference", "redoxi_works::inference::RedoxiModelInference")
    {
    }

    // keep the loader alive during the lifetime of the class
    pluginlib::ClassLoader<RedoxiModelInference> loader;

    inline static const std::string PreprocessInfoKey = "preprocess_info";
};
} // namespace redoxi_works::inference::yolo8
