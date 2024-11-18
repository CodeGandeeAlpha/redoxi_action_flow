#include <pluginlib/class_loader.hpp>
#include <typeinfo>
#include <filesystem>

#include <redoxi_dnn_models/Yolo8Pose.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>

#include <redoxi_dnn_models/Yolo8Preprocessor.hpp>


#define ENABLE_DEBUG_OUTPUT

#ifdef ENABLE_DEBUG_OUTPUT
#    include <xtensor/xnpy.hpp>
#endif

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
                                const std::vector<cv::Mat> &rgb_images)
{
    // all images should be of the same size and same number of channels
    for (const auto &image : rgb_images) {
        if (image.empty()) {
            // all images must be non-empty
            return -1;
        }
        if (image.size() != rgb_images[0].size()) {
            // all images must be of the same size
            return -1;
        }
        if (image.channels() != rgb_images[0].channels()) {
            // all images must have the same number of channels
            return -1;
        }
    }

    yolo8::Yolo8Preprocessor preprocessor;
    yolo8::Yolo8PreprocessorConfig config;
    auto [expected_batch_size, expected_num_channels, expected_height, expected_width] = get_model_input_shape_nchw();
    config.model_input_image_size = cv::Size(expected_width, expected_height);
    preprocessor.init(config);

    // check batch size
    if (expected_batch_size > 0 && (int64_t)rgb_images.size() != expected_batch_size) {
        RDX_RAISE_ERROR("Expecting {} images, got {}", expected_batch_size, rgb_images.size());
    }
    int64_t batch_size = (int64_t)rgb_images.size();

    // create a 4d tensor to hold all images
    auto input_dtype = get_model_input_dtype();
    if (input_dtype != "float32") {
        RDX_RAISE_ERROR("Unsupported model input dtype: {}", input_dtype);
    }

    // for each image, do preprocess
    std::vector<float> tensor_data(batch_size * expected_num_channels * expected_height * expected_width, 0.0f);
    std::vector<yolo8::ImagePreprocessInfo> preprocess_info(batch_size);
    for (int64_t i = 0; i < batch_size; ++i) {
        preprocessor.preprocess(tensor_data.data() + i * expected_num_channels * expected_height * expected_width,
                                &preprocess_info[i], rgb_images[i], "rgb");
    }

    // copy the tensor to the model input
    auto port_data = model_inout_data->get_input_port_data(m_model_input_info->get_name());
    port_data->set_tensor_data(tensor_data.data(), {batch_size, expected_num_channels, expected_height, expected_width});

    auto any_data = std::make_shared<std::any>(preprocess_info);
    model_inout_data->set_any_data(Impl::PreprocessInfoKey, any_data);

    return 0;
}

// set the input images to the model
// the images must be of the same size, in RGB format
int Yolo8Pose::set_input_images_v1(InferenceInOutData::Ptr model_inout_data,
                                   const std::vector<cv::Mat> &rgb_images)
{
    // all images should be of the same size and same number of channels
    for (const auto &image : rgb_images) {
        if (image.empty()) {
            // all images must be non-empty
            return -1;
        }
        if (image.size() != rgb_images[0].size()) {
            // all images must be of the same size
            return -1;
        }
        if (image.channels() != rgb_images[0].channels()) {
            // all images must have the same number of channels
            return -1;
        }
    }

    // create a 4d tensor to hold all images
    auto [expected_batch_size, expected_num_channels, expected_height, expected_width] = get_model_input_shape_nchw();
    if (expected_batch_size > 0 && (int64_t)rgb_images.size() != expected_batch_size) {
        RDX_RAISE_ERROR("Expecting {} images, got {}", expected_batch_size, rgb_images.size());
    }
    int64_t batch_size = rgb_images.size();
    auto input_dtype = get_model_input_dtype();
    cv::Size model_input_size(expected_width, expected_height);

    // this will be attached to the model inout data, for post-processing after inference
    yolo8::ImagePreprocessInfo::List preprocess_info(batch_size);

    // create a tensor to hold the images
    if (input_dtype == "float32") {
        // FIXME: for fixed size input, we can pre-allocate the tensor, but for now, we just allocate it here

        // NCHW tensor
        auto tensor = std::vector<float>(batch_size * expected_num_channels * expected_height * expected_width, 0.0f);

        // resize the images to the model input size
        for (int64_t i = 0; i < batch_size; ++i) {
            auto &pinfo = preprocess_info[i];
            pinfo.source_image_size = rgb_images[i].size();
            pinfo.roi_in_source_image = {0, 0, rgb_images[i].cols, rgb_images[i].rows};
            pinfo.model_input_image_size = model_input_size;

            // resize the image to the model input size
            cv::Mat resized_image(model_input_size, rgb_images[i].type());
            cv::Size dst_size = redoxi_works::image_utils::compute_resize_to_fit_and_keep_aspect_ratio(
                rgb_images[i].size(), model_input_size);
            {
                // roi in the destination image where the resized image will be placed
                cv::Rect roi(0, 0, dst_size.width, dst_size.height);

                // resize the image to the destination size, filling the rest with zeros
                cv::resize(rgb_images[i], resized_image(roi), dst_size);
                pinfo.roi_in_model_input_image = roi;
            }

            // convert to float and normalize to [0,1]
            if (resized_image.channels() == 3) {
                resized_image.convertTo(resized_image, CV_32FC3, 1.0f / 255.0f);
            } else if (resized_image.channels() == 1) {
                resized_image.convertTo(resized_image, CV_32FC1, 1.0f / 255.0f);
            } else {
                RDX_RAISE_ERROR("Unsupported number of channels: {}", resized_image.channels());
            }

            // convert to the expected number of channels
            if (expected_num_channels == 1 && resized_image.channels() == 3) {
                // if input is 3 channel and model is single channel, we need to convert the image to gray scale
                cv::cvtColor(resized_image, resized_image, cv::COLOR_RGB2GRAY);
            } else if (expected_num_channels == 3 && resized_image.channels() == 1) {
                // if input is single channel and model is 3 channel, we need to replicate the image to 3 channels
                cv::cvtColor(resized_image, resized_image, cv::COLOR_GRAY2RGB);
            } else if (expected_num_channels != resized_image.channels()) {
                // if the number of channels does not match, return error
                RDX_RAISE_ERROR("Input image channel ({}) does not match model input channel ({}), and cannot be converted",
                                resized_image.channels(), expected_num_channels);
            }

            // split the image into channels
            std::vector<cv::Mat> channels;
            cv::split(resized_image, channels);

#ifdef ENABLE_DEBUG_OUTPUT
            {
                //! Print max value of resized_image
                double min_val, max_val;
                cv::minMaxLoc(resized_image, &min_val, &max_val);
                RDX_INFO_DEV(nullptr, __func__, "Image {} max value: {}", i, max_val);
                const char *debug_dir = "/soft/workspace/code/psf_ros2_ws/tmp/output";
                if (!fs::exists(debug_dir)) {
                    fs::create_directories(debug_dir);
                }
                fs::path fn_debug = fs::path(debug_dir) / fs::path(fmt::format("pose-input-{}.png", i));
                cv::Mat uint8_image;
                resized_image.convertTo(uint8_image, CV_8UC3, 255.0);
                cv::imwrite(fn_debug.string(), uint8_image);
            }
#endif

            // copy the channels to the tensor
            for (int64_t c = 0; c < expected_num_channels; ++c) {
                std::copy(channels[c].begin<float>(), channels[c].end<float>(),
                          tensor.begin() + i * expected_num_channels * expected_height * expected_width + c * expected_height * expected_width);
            }

#ifdef ENABLE_DEBUG_OUTPUT
            {
                // write the tensor to numpy file
                std::array<size_t, 4> tensor_shape = {
                    (size_t)batch_size, (size_t)expected_num_channels,
                    (size_t)expected_height, (size_t)expected_width};
                xt::xarray<float> tensor_xarray = xt::adapt(tensor, tensor_shape);
                xt::dump_npy(fmt::format("/soft/workspace/code/psf_ros2_ws/tmp/output/pose-input-{}.npy", i), tensor_xarray);
            }
#endif
        }

        // copy the tensor to the model input
        auto port_data = model_inout_data->get_input_port_data(m_model_input_info->get_name());
        port_data->set_tensor_data(tensor.data(), {batch_size, expected_num_channels, expected_height, expected_width});

        auto any_data = std::make_shared<std::any>(preprocess_info);
        model_inout_data->set_any_data(Impl::PreprocessInfoKey, any_data);

    } else if (input_dtype == "uint8") {
        throw std::runtime_error("uint8 input is not supported yet");
    } else {
        RDX_RAISE_ERROR("Unsupported model input dtype: {}", input_dtype);
    }

    return 0;
}

std::vector<Yolo8Pose::SingleImageOutput>
    Yolo8Pose::get_output_detections(
        InferenceInOutData::Ptr model_inout_data,
        double confidence_thres) const
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

    // get dimensions
    int64_t batch_size = tensor_shape[0];
    int64_t num_values = tensor_shape[1];
    int64_t num_objects = tensor_shape[2];

    // each object has 4 bbox values (x_center,y_center,width,height) + score + keypoints (x,y,score)
    int64_t num_keypoints = (num_values - 5) / 3;

    // get preprocess info
    auto any_data = model_inout_data->get_any_data(Impl::PreprocessInfoKey);
    if (!any_data) {
        RDX_RAISE_ERROR("Preprocess info not found");
    }
    auto preprocess_info = std::any_cast<yolo8::ImagePreprocessInfo::List>(*any_data);

    //! Process each image in batch
    std::vector<SingleImageOutput> outputs(batch_size);
    for (int64_t b = 0; b < batch_size; b++) {
        auto &detections = outputs[b].objects;

        cv::Mat _object_data(
            num_values, num_objects, CV_32FC1,
            const_cast<float *>(tensor_data + b * num_objects * num_values));

        // each row is the inference result for one object
        // format as (x_center,y_center,width,height,score,kp1_x,kp1_y,kp1_score,kp2_x,kp2_y,kp2_score,...)
        cv::Mat object_data;
        cv::transpose(_object_data, object_data);

        //! Process each potential object
        for (int64_t obj = 0; obj < num_objects; obj++) {
            //! Get row data for this object
            const float *obj_data = object_data.ptr<float>(obj);

            //! Get confidence score
            float score = obj_data[4];

            //! Skip if below confidence threshold
            if (score < confidence_thres) {
                continue;
            }

            //! Extract bounding box
            float x_center = obj_data[0];
            float y_center = obj_data[1];
            float width = obj_data[2];
            float height = obj_data[3];

            //! Create detection object
            DetectedObject det;
            det.xywh = {x_center - width / 2, y_center - height / 2, width, height};
            det.score = score;
            det.class_id = 0; // Only person class for pose model

            //! Extract keypoints
            det.keypoints.resize(num_keypoints);
            for (int64_t k = 0; k < num_keypoints; k++) {
                int64_t kp_offset = 5 + k * 3;
                det.keypoints[k].xy = {obj_data[kp_offset], obj_data[kp_offset + 1]};
                det.keypoints[k].score = obj_data[kp_offset + 2];
            }

            detections.push_back(std::move(det));
        }

        {
            // transform all coordinates from model input space to source image space
            auto &pinfo = preprocess_info[b];
            auto dx_model = -pinfo.roi_in_model_input_image.x;
            auto dy_model = -pinfo.roi_in_model_input_image.y;
            auto scale_x = pinfo.roi_in_source_image.width / (float)pinfo.roi_in_model_input_image.width;
            auto scale_y = pinfo.roi_in_source_image.height / (float)pinfo.roi_in_model_input_image.height;
            auto dx_source = pinfo.roi_in_source_image.x;
            auto dy_source = pinfo.roi_in_source_image.y;

            for (auto &det : detections) {
                det.xywh[0] = (det.xywh[0] + dx_model) * scale_x + dx_source;
                det.xywh[1] = (det.xywh[1] + dy_model) * scale_y + dy_source;
                det.xywh[2] *= scale_x;
                det.xywh[3] *= scale_y;

                // transform keypoints
                for (auto &kp : det.keypoints) {
                    kp.xy[0] = (kp.xy[0] + dx_model) * scale_x + dx_source;
                    kp.xy[1] = (kp.xy[1] + dy_model) * scale_y + dy_source;
                }
            }
        }
    }

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

} // namespace redoxi_works::inference
