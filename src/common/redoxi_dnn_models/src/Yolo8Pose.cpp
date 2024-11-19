#include <pluginlib/class_loader.hpp>
#include <typeinfo>
#include <filesystem>

#include <redoxi_dnn_models/Yolo8Pose.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_dnn_models/Yolo8Preprocessor.hpp>
#include <redoxi_dnn_models/Yolo8Postprocessor.hpp>

#define ENABLE_DEBUG_OUTPUT
#ifdef ENABLE_DEBUG_OUTPUT
#    include <xtensor/xnpy.hpp>
#endif

#define USE_YOLOR8_POSTPROCESSOR

namespace fs = std::filesystem;
namespace redoxi_works::inference
{

struct Yolo8Pose::Impl {
    Impl()
        : loader("redoxi_inference", "redoxi_works::inference::RedoxiModelInference")
    {
    }

    // keep the loader alive during the lifetime of the class
    pluginlib::ClassLoader<RedoxiModelInference> loader;

    inline static const std::string PreprocessInfoKey = "preprocess_info";
};

Yolo8Pose::Yolo8Pose()
    : m_impl(std::make_shared<Impl>())
{
}

KeyValueStore::Ptr Yolo8Pose::create_init_params()
{
    return std::make_shared<InitConfig_t>();
}

InferenceInOutData::Ptr Yolo8Pose::create_inference_inout_data()
{
    if (m_model) {
        return m_model->create_inference_inout_data();
    }
    return nullptr;
}

ModelPortInfo::ConstPtrMap Yolo8Pose::get_input_port_infos() const
{
    if (m_model) {
        return m_model->get_input_port_infos();
    }
    return {};
}

ModelPortInfo::ConstPtrMap Yolo8Pose::get_output_port_infos() const
{
    if (m_model) {
        return m_model->get_output_port_infos();
    }
    return {};
}

int Yolo8Pose::open(KeyValueStore::Ptr params)
{
    // Open the model with provided parameters
    auto _params = std::dynamic_pointer_cast<InitConfig_t>(params);
    if (!_params) {
        RDX_RAISE_ERROR("Invalid parameters, expected Yolo8PoseConfig, got {}",
                        typeid(params.get()).name());
    }

    // Create onnx model instance
    auto model = m_impl->loader.createSharedInstance("redoxi_works::inference::onnx::OnnxModelInference");
    if (!model) {
        RDX_RAISE_ERROR("Failed to create onnx model instance");
    }

    // Get model parameters
    std::string model_path, device_type;
    int64_t device_index;
    _params->get_string(&model_path, common_config_keys::ModelPath);
    _params->get_string(&device_type, common_config_keys::DeviceType);
    _params->get_int(&device_index, common_config_keys::DeviceIndex);
    RDX_INFO_DEV(nullptr, __func__, "Opening model from {}, device type {}, device index {}",
                 model_path, device_type, device_index);

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

bool Yolo8Pose::is_open() const
{
    if (!m_model)
        return false;
    return m_model->is_open();
}

int Yolo8Pose::close()
{
    if (!m_model)
        return 0;
    return m_model->close();
}

KeyValueStore::ConstPtr Yolo8Pose::get_model_metadata() const
{
    return m_init_params;
}

KeyValueStore::ConstPtr Yolo8Pose::get_inference_metadata() const
{
    return m_init_params;
}

int Yolo8Pose::do_inference(InferenceInOutData::Ptr inout_data)
{
    // just dispatch to the inner model
    if (!m_model)
        return -1;
    return m_model->do_inference(inout_data);
}

int Yolo8Pose::set_input_images(InferenceInOutData::Ptr model_inout_data,
                                const std::vector<cv::Mat> &images,
                                const std::string &image_format)
{
    // all images should be of the same size and same number of channels
    // for (const auto &image : images) {
    //     if (image.empty()) {
    //         // all images must be non-empty
    //         return -1;
    //     }
    //     if (image.size() != images[0].size()) {
    //         // all images must be of the same size
    //         return -1;
    //     }
    //     if (image.channels() != images[0].channels()) {
    //         // all images must have the same number of channels
    //         return -1;
    //     }
    // }

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

std::vector<Yolo8Pose::SingleImageOutput>
    Yolo8Pose::get_output_detections(
        InferenceInOutData::Ptr model_inout_data,
        const OutputConfig_t &config) const
{
    // get the output port data
    auto output_port_name = m_model_output_info->get_name();
    auto port_data = model_inout_data->get_output_port_data(output_port_name);
    if (!port_data) {
        RDX_RAISE_ERROR("Output port data not found for port: {}", output_port_name);
    }
    if (port_data->get_dtype_str() != "float32") {
        RDX_RAISE_ERROR("Unsupported model output dtype: {}, required float32", port_data->get_dtype_str());
    }

    // expected output tensor (batch_size, 4+num_keypoints*3, num_objects)
    auto tensor_shape = port_data->get_shape();
    const float *tensor_data = nullptr;
    port_data->get_tensor_data(&tensor_data);

    // check tensor shape
    if (tensor_shape.size() != 3) {
        RDX_RAISE_ERROR("Invalid output tensor shape: expected 3 dimensions, got {}", tensor_shape.size());
    }
    RDX_INFO_DEV(nullptr, __func__, "Output tensor shape: ({},{},{})", tensor_shape[0], tensor_shape[1], tensor_shape[2]);

    // get preprocess info
    auto any_data = model_inout_data->get_any_data(Impl::PreprocessInfoKey);
    if (!any_data) {
        RDX_RAISE_ERROR("Preprocess info not found");
    }
    auto preprocess_info = std::any_cast<yolo8::ImagePreprocessInfo::List>(*any_data);

    yolo8::Yolo8Postprocessor postprocessor;
    postprocessor.init(config);

    yolo8::SingleImageOutput::List outputs;
    postprocessor.postprocess(&outputs, tensor_data,
                              std::array<int64_t, 3>{tensor_shape[0], tensor_shape[1], tensor_shape[2]},
                              preprocess_info);
    return outputs;
}

std::array<int64_t, 4> Yolo8Pose::get_model_input_shape_nchw() const
{
    std::array<int64_t, 4> shape{0, 0, 0, 0};
    auto shape_vec = m_model_input_info->get_shape();
    std::copy(shape_vec.begin(), shape_vec.end(), shape.begin());
    return shape;
}

std::string Yolo8Pose::get_model_input_dtype() const
{
    return m_model_input_info->get_dtype_str();
}

std::array<int64_t, 3> Yolo8Pose::get_model_output_shape_nchw() const
{
    std::array<int64_t, 3> shape{0, 0, 0};
    auto shape_vec = m_model_output_info->get_shape();
    std::copy(shape_vec.begin(), shape_vec.end(), shape.begin());
    return shape;
}

std::string Yolo8Pose::get_model_output_dtype() const
{
    return m_model_output_info->get_dtype_str();
}

std::vector<std::pair<int, int>>
    Yolo8Pose::get_keypoint_connections() const
{
    static const std::vector<std::pair<int, int>> connections = {
        {15, 13}, {13, 11}, {16, 14}, {14, 12}, {11, 12}, {5, 11}, {6, 12}, {5, 6}, {5, 7}, {6, 8}, {7, 9}, {8, 10}, {1, 2}, {0, 1}, {0, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}};
    return connections;
}

} // namespace redoxi_works::inference
