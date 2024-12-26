#include <redoxi_dnn_models/yolo8/Yolo8ModelBase.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8Preprocessor.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8Postprocessor.hpp>
#include <redoxi_dnn_models/yolo8/Yolo8ModelBaseImpl.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <filesystem>
#define ENABLE_DEBUG_OUTPUT
#ifdef ENABLE_DEBUG_OUTPUT
#    include <xtensor/xnpy.hpp>
#endif


namespace fs = std::filesystem;
namespace redoxi_works::inference::yolo8
{

Yolo8ModelBase::Yolo8ModelBase()
    : m_impl(std::make_shared<Impl>())
{
}

KeyValueStore::Ptr Yolo8ModelBase::create_init_params()
{
    return std::make_shared<InitConfig_t>();
}

InferenceInOutData::Ptr Yolo8ModelBase::create_inference_inout_data()
{
    if (m_model) {
        return m_model->create_inference_inout_data();
    }
    return nullptr;
}

ModelPortInfo::ConstPtrMap Yolo8ModelBase::get_input_port_infos() const
{
    if (m_model) {
        return m_model->get_input_port_infos();
    }
    return {};
}

ModelPortInfo::ConstPtrMap Yolo8ModelBase::get_output_port_infos() const
{
    if (m_model) {
        return m_model->get_output_port_infos();
    }
    return {};
}

int Yolo8ModelBase::open(KeyValueStore::Ptr params)
{
    // Open the model with provided parameters
    auto _params = std::dynamic_pointer_cast<InitConfig_t>(params);
    if (!_params) {
        RDX_RAISE_ERROR("Invalid parameters, expected Yolo8PoseModelConfig, got {}",
                        typeid(params.get()).name());
    }

    // Get model parameters
    std::string model_path, device_type;
    int64_t device_index;
    _params->get_string(&model_path, common_config_keys::ModelPath);
    _params->get_string(&device_type, common_config_keys::DeviceType);
    _params->get_int(&device_index, common_config_keys::DeviceIndex);
    RDX_INFO_DEV(nullptr, __func__, "Opening model from {}, device type {}, device index {}",
                 model_path, device_type, device_index);

    // check if the model file exists
    if (!fs::exists(model_path)) {
        RDX_RAISE_ERROR("[f={}()] Model file does not exist: {}", __func__, model_path);
        return -1;
    }

    // Create onnx model instance
    // TODO: add rknn model
    std::shared_ptr<RedoxiModelInference> model;
    if (device_type == common_device_types::RKNPU) {
        // rknn model, where the model itself should be .rknn file
        model = m_impl->loader.createSharedInstance("redoxi_works::inference::rknn::RknnModelInference");

        // check if the model file has .rknn extension
        if (fs::path(model_path).extension() != ".rknn") {
            RDX_RAISE_ERROR("[f={}()] Invalid model file extension for RKNPU device, expected .rknn: {}", __func__, model_path);
            return -1;
        }
    } else {
        // default to onnx
        model = m_impl->loader.createSharedInstance("redoxi_works::inference::onnx::OnnxModelInference");

        // check if the model file has .onnx extension
        if (fs::path(model_path).extension() != ".onnx") {
            RDX_RAISE_ERROR("[f={}()] Invalid model file extension for ONNX device, expected .onnx: {}", __func__, model_path);
            return -1;
        }
    }

    if (!model) {
        RDX_RAISE_ERROR("Failed to create model instance");
    }

    // transfer the parameters to the inner model
    auto inner_params = model->create_init_params();
    inner_params->set_string(common_config_keys::ModelPath, model_path);
    inner_params->set_string(common_config_keys::DeviceType, device_type);
    inner_params->set_int(common_config_keys::DeviceIndex, device_index);
    if (model->open(inner_params) != 0) {
        // reset the model pointer if the model failed to open
        RDX_RAISE_ERROR("Failed to open the model");
    }

    // get the model input and output dtype
    //! get the model input and output dtype from first port
    auto input_ports = model->get_input_port_infos();
    if (input_ports.size() != 1) {
        RDX_RAISE_ERROR("Expecting exactly one input port, got {}", input_ports.size());
    }
    m_model_input_info = input_ports.begin()->second;

    //! check input shape is 4D (NCHW)
    if (m_model_input_info->get_shape().size() != 4) {
        RDX_RAISE_ERROR("Invalid input tensor shape: expected 4 dimensions (NCHW), got {}",
                        m_model_input_info->get_shape().size());
    }

    // get the model output dtype
    auto output_ports = model->get_output_port_infos();
    if (output_ports.size() != 1) {
        RDX_RAISE_ERROR("Expecting exactly one output port, got {}", output_ports.size());
    }
    m_model_output_info = output_ports.begin()->second;

    //! check output shape is 3D
    if (m_model_output_info->get_shape().size() != 3) {
        RDX_RAISE_ERROR("Invalid output tensor shape: expected 3 dimensions, got {}",
                        m_model_output_info->get_shape().size());
    }

    //! check output dtype is float32
    if (m_model_output_info->get_dtype_str() != "float32") {
        RDX_RAISE_ERROR("Invalid output tensor dtype: expected float32, got {}",
                        m_model_output_info->get_dtype_str());
    }

    // set the model pointer
    RDX_INFO_DEV(nullptr, __func__, "{}", "Model opened successfully");
    RDX_INFO_DEV(nullptr, __func__, "Model input dtype: {}, input size (N,C,H,W): ({},{},{},{}), output dtype: {}",
                 m_model_input_info->get_dtype_str(), m_model_input_info->get_shape()[0],
                 m_model_input_info->get_shape()[1], m_model_input_info->get_shape()[2],
                 m_model_input_info->get_shape()[3], m_model_output_info->get_dtype_str());
    m_model = model;
    m_init_params = _params;

    return 0;
}

bool Yolo8ModelBase::is_open() const
{
    if (!m_model)
        return false;
    return m_model->is_open();
}

int Yolo8ModelBase::close()
{
    if (!m_model)
        return 0;
    return m_model->close();
}

KeyValueStore::ConstPtr Yolo8ModelBase::get_model_metadata() const
{
    return m_init_params;
}

KeyValueStore::ConstPtr Yolo8ModelBase::get_inference_metadata() const
{
    return m_init_params;
}

int Yolo8ModelBase::do_inference(InferenceInOutData::Ptr inout_data)
{
    // just dispatch to the inner model
    if (!m_model)
        return -1;
    return m_model->do_inference(inout_data);
}

int Yolo8ModelBase::set_input_images(InferenceInOutData::Ptr model_inout_data,
                                     const std::vector<cv::Mat> &images,
                                     const std::string &image_format)
{
    // all images should be of the same size
    for (const auto &image : images) {
        if (image.empty()) {
            // all images must be non-empty
            return -1;
        }
        if (image.size() != images[0].size()) {
            // all images must be of the same size
            return -1;
        }
    }

    yolo8::Yolo8Preprocessor preprocessor;
    yolo8::Yolo8PreprocessorConfig config;
    auto [expected_batch_size, expected_num_channels, expected_height, expected_width] = get_model_input_shape_nchw();
    config.model_input_image_size = cv::Size(expected_width, expected_height);
    preprocessor.init(config);

    // the model allows for dynamic batch size? if not, check batch size
    if (expected_batch_size > 0 && (int64_t)images.size() != expected_batch_size) {
        RDX_RAISE_ERROR("Expecting {} images, got {}", expected_batch_size, images.size());
    }
    int64_t batch_size = (int64_t)images.size();

    // create a 4d tensor to hold all images
    auto input_dtype = get_model_input_dtype();
    if (input_dtype != "float32") {
        RDX_RAISE_ERROR("Unsupported model input dtype: {}", input_dtype);
    }

    // for each image, do preprocess
    std::vector<float> tensor_data(batch_size * expected_num_channels * expected_height * expected_width, 0.0f);
    yolo8::ImagePreprocessInfo::List preprocess_info(batch_size);
    preprocessor.preprocess(tensor_data.data(), &preprocess_info, images, image_format);

    // copy the tensor to the model input
    auto port_data = model_inout_data->get_input_port_data(m_model_input_info->get_name());
    port_data->set_tensor_data(tensor_data.data(), {batch_size, expected_num_channels, expected_height, expected_width});

    // attach the preprocess info to the model input
    auto any_data = std::make_shared<std::any>(preprocess_info);
    model_inout_data->set_any_data(Impl::PreprocessInfoKey, any_data);

    return 0;
}

std::array<int64_t, 4> Yolo8ModelBase::get_model_input_shape_nchw() const
{
    std::array<int64_t, 4> shape{0, 0, 0, 0};
    auto shape_vec = m_model_input_info->get_shape();
    std::copy(shape_vec.begin(), shape_vec.end(), shape.begin());
    return shape;
}

std::string Yolo8ModelBase::get_model_input_dtype() const
{
    return m_model_input_info->get_dtype_str();
}

std::array<int64_t, 3> Yolo8ModelBase::get_model_output_shape() const
{
    std::array<int64_t, 3> shape{0, 0, 0};
    auto shape_vec = m_model_output_info->get_shape();
    std::copy(shape_vec.begin(), shape_vec.end(), shape.begin());
    return shape;
}

std::string Yolo8ModelBase::get_model_output_dtype() const
{
    return m_model_output_info->get_dtype_str();
}

} // namespace redoxi_works::inference::yolo8