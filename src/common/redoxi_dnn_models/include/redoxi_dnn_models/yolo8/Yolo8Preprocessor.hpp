#pragma once

#include <redoxi_dnn_models/redoxi_dnn_models.hpp>
#include <redoxi_dnn_models/detection_types.hpp>
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
    virtual void preprocess(
        float *output_tensor_nchw,
        ImagePreprocessInfo::List *output_preprocess_info,
        const std::vector<cv::Mat> &input_images,
        const std::string &image_format) const;

  protected:
    // preprocess the input image to the model input image
    // the output_tensor_chw is a pointer to the tensor data, which has the shape of [num_channels, height, width]
    // where the size is the model input image size
    // image format can be "rgb8" or "bgr8" or "mono8", following the ros2 sensor_msgs::image_encodings convention
    virtual void preprocess(
        float *output_tensor_chw,
        ImagePreprocessInfo *output_preprocess_info,
        const cv::Mat &input_image,
        const std::string &image_format) const;

  protected:
    Yolo8PreprocessorConfig m_config;
};
} // namespace redoxi_works::inference::yolo8
