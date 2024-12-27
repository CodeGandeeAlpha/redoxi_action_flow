#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_dnn_models/detection_types.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

namespace redoxi_works::inference::yolo8
{
using namespace redoxi_works::inference::detection::types;

class Yolo8PreprocessorConfig
{
  public:
    cv::Size model_input_image_size;
    cv::InterpolationFlags interp_method = cv::INTER_LINEAR;
};

class Yolo8Preprocessor
{
  public:
    // the model always expect 3 channels
    inline constexpr static int64_t ModelInputNumChannels = 3;

    // the model always expect the input image format to be "rgb8"
    inline constexpr static const char *ModelInputImageFormat = sensor_msgs::image_encodings::RGB8;

    virtual ~Yolo8Preprocessor() = default;
    virtual void init(const Yolo8PreprocessorConfig &config);

    // batch version of the preprocess function
    // images should have the same format, size, and number of channels
    // virtual void preprocess(
    //     float *output_tensor_nchw,
    //     ImagePreprocessInfo::List *output_preprocess_info,
    //     const std::vector<cv::Mat> &input_images,
    //     const std::string &image_format) const;

    // batch version of the preprocess function
    template <typename T>
    requires std::is_floating_point_v<T> || std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>
    void preprocess(
        T *output_tensor_data,
        inference::TensorFormat output_tensor_format,
        ImagePreprocessInfo::List *output_preprocess_info,
        const std::vector<cv::Mat> &input_images,
        const std::string &image_format) const;

  protected:
    // preprocess the input image to the model input image
    // the output_tensor_chw is a pointer to the tensor data, which has the shape of [num_channels, height, width]
    // where the size is the model input image size
    // image format can be "rgb8" or "bgr8" or "mono8", following the ros2 sensor_msgs::image_encodings convention
    // virtual void preprocess(
    //     float *output_tensor_chw,
    //     ImagePreprocessInfo *output_preprocess_info,
    //     const cv::Mat &input_image,
    //     const std::string &image_format) const;


    /**
     * @brief Preprocess the input image to the model input image
     * @param output_tensor_data Pointer to the tensor data with shape [num_channels, height, width]
     * @param output_tensor_format Format of the output tensor
     * @param output_preprocess_info Output preprocessing information
     * @param input_image Input image to preprocess
     * @param image_encoding Image encoding ("rgb8", "bgr8", or "mono8") following ROS2 sensor_msgs::image_encodings
     * @note The output tensor size matches the model input image size
     */
    template <typename T>
    requires std::is_floating_point_v<T> || std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>
    void preprocess(
        T *output_tensor_data,
        inference::TensorFormat output_tensor_format,
        ImagePreprocessInfo *output_preprocess_info,
        const cv::Mat &input_image,
        const std::string &image_encoding) const;

  protected:
    Yolo8PreprocessorConfig m_config;
};

template <typename T>
requires std::is_floating_point_v<T> || std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>
void Yolo8Preprocessor::preprocess(
    T *output_tensor_data,
    inference::TensorFormat output_tensor_format,
    ImagePreprocessInfo *output_preprocess_info,
    const cv::Mat &input_image,
    const std::string &image_encoding) const
{
    ImagePreprocessInfo pinfo;

    // get image sizes
    cv::Size source_image_size(input_image.cols, input_image.rows);
    cv::Size model_input_image_size = m_config.model_input_image_size;
    pinfo.source_image_size = source_image_size;
    pinfo.model_input_image_size = model_input_image_size;
    pinfo.roi_in_source_image = cv::Rect(0, 0, source_image_size.width, source_image_size.height);

    // compute the roi in the model input image
    auto dst_roi_size = image_utils::compute_resize_to_fit_and_keep_aspect_ratio(
        source_image_size, model_input_image_size);
    pinfo.roi_in_model_input_image = cv::Rect(0, 0, dst_roi_size.width, dst_roi_size.height);

    // resize the image to model input size, filling part of the image with 0 if necessary
    cv::Mat resized_image(model_input_image_size, input_image.type(), cv::Scalar(0));
    cv::resize(input_image, resized_image(pinfo.roi_in_model_input_image), dst_roi_size, 0, 0, m_config.interp_method);

    //! Convert image type based on output tensor type
    int output_cv_depth = 0;
    if constexpr (std::is_same_v<T, float>) {
        // For float output, convert integer image to float32 and scale
        if (resized_image.type() == CV_8UC1 || resized_image.type() == CV_8UC3) {
            resized_image.convertTo(resized_image, CV_32F, 1.0 / 255.0);
        }
        output_cv_depth = CV_32F;
    } else if constexpr (std::is_same_v<T, double>) {
        // For double output, convert integer image to float64 and scale
        if (resized_image.type() == CV_8UC1 || resized_image.type() == CV_8UC3) {
            resized_image.convertTo(resized_image, CV_64F, 1.0 / 255.0);
        }
        output_cv_depth = CV_64F;
    } else if constexpr (std::is_same_v<T, uint8_t>) {
        // For uint8 output, convert float image to uint8 and scale up
        if (resized_image.type() == CV_32F || resized_image.type() == CV_64F) {
            resized_image.convertTo(resized_image, CV_8U, 255.0);
        }
        output_cv_depth = CV_8U;
    } else if constexpr (std::is_same_v<T, uint16_t>) {
        // For uint16 output, convert float image to uint16 and scale up
        if (resized_image.type() == CV_32F || resized_image.type() == CV_64F) {
            resized_image.convertTo(resized_image, CV_16U, 65535.0);
        }
        output_cv_depth = CV_16U;
    }

    if (output_tensor_format == inference::TensorFormat::NCHW) {
        // convert to CHW format, by splitting the image into channels
        std::vector<cv::Mat> channels;
        cv::split(resized_image, channels);

        // handle different image formats
        std::vector<int> channel_map;
        if (image_encoding == sensor_msgs::image_encodings::RGB8) {
            channel_map = {0, 1, 2}; // RGB -> RGB, read the channels in the order of R, G, B
        } else if (image_encoding == sensor_msgs::image_encodings::BGR8) {
            channel_map = {2, 1, 0}; // BGR -> RGB, read the channels in the order of B, G, R
        } else if (image_encoding == sensor_msgs::image_encodings::MONO8) {
            channel_map = {0, 0, 0}; // Grayscale, read the channels in the order of 0, 0, 0
        } else {
            throw std::runtime_error("Unsupported image format: " + image_encoding);
        }

        // copy the channels to the output tensor in CHW format
        // will always write the channels in the order of R, G, B
        for (size_t c = 0; c < channels.size(); c++) {
            // get the index of the input channel, and then get the channel data
            // make sure always read in the order of R, G, B
            int index_input = channel_map[c];
            auto channel_mat = channels[index_input];

            // write the channel data to the output tensor in CHW format
            int index_output = c;
            auto *out_ptr = output_tensor_data + index_output * model_input_image_size.height * model_input_image_size.width;
            channel_mat.copyTo(cv::Mat(model_input_image_size, output_cv_depth, out_ptr));
        }
    } else if (output_tensor_format == inference::TensorFormat::NHWC) {
        // copy the image to the output tensor in NHWC format
        // For NHWC format, need to specify the number of channels in the output matrix
        cv::Mat output_mat(model_input_image_size, CV_MAKETYPE(output_cv_depth, ModelInputNumChannels), output_tensor_data);
        resized_image.copyTo(output_mat);
    }

    // fill preprocess info if requested
    if (output_preprocess_info) {
        *output_preprocess_info = pinfo;
    }
}

template <typename T>
requires std::is_floating_point_v<T> || std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t>
void Yolo8Preprocessor::preprocess(
    T *output_tensor_data,
    inference::TensorFormat output_tensor_format,
    ImagePreprocessInfo::List *output_preprocess_info,
    const std::vector<cv::Mat> &input_images,
    const std::string &image_format) const
{
    //! All images must be of the same size
    if (input_images.empty()) {
        throw std::runtime_error("No input images provided");
    }

    //! Just call the single image version for each image in the batch
    ImagePreprocessInfo::List pinfo_list(input_images.size());
    auto model_width = m_config.model_input_image_size.width;
    auto model_height = m_config.model_input_image_size.height;

    for (size_t i = 0; i < input_images.size(); i++) {
        auto &pinfo = pinfo_list[i];
        T *data_ptr = output_tensor_data + i * ModelInputNumChannels * model_height * model_width;
        preprocess(
            data_ptr,
            output_tensor_format,
            &pinfo,
            input_images[i],
            image_format);
    }

    if (output_preprocess_info) {
        *output_preprocess_info = pinfo_list;
    }
}

} // namespace redoxi_works::inference::yolo8
