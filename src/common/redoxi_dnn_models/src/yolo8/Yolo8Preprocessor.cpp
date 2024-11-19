#include <redoxi_dnn_models/yolo8/Yolo8Preprocessor.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>

namespace redoxi_works::inference::yolo8
{
void Yolo8Preprocessor::init(const Yolo8PreprocessorConfig &config)
{
    m_config = config;
}

void Yolo8Preprocessor::preprocess(
    float *output_tensor_nchw,
    ImagePreprocessInfo::List *output_preprocess_info,
    const std::vector<cv::Mat> &input_images,
    const std::string &image_format) const
{
    // all images must be of the same size
    if (input_images.empty()) {
        throw std::runtime_error("No input images provided");
    }

    // just call the single image version for each image in the batch
    ImagePreprocessInfo::List pinfo_list(input_images.size());
    auto model_width = m_config.model_input_image_size.width;
    auto model_height = m_config.model_input_image_size.height;
    for (size_t i = 0; i < input_images.size(); i++) {
        auto &pinfo = pinfo_list[i];
        float *data_ptr = output_tensor_nchw + i * ModelInputNumChannels * model_height * model_width;
        preprocess(
            data_ptr,
            &pinfo,
            input_images[i],
            image_format);
    }

    if (output_preprocess_info) {
        *output_preprocess_info = pinfo_list;
    }
}

void Yolo8Preprocessor::preprocess(
    float *output_tensor_chw,
    ImagePreprocessInfo *output_preprocess_info,
    const cv::Mat &input_image,
    const std::string &image_format) const
{
    ImagePreprocessInfo pinfo;

    // get image sizes
    cv::Size source_image_size(input_image.cols, input_image.rows);
    cv::Size model_input_image_size = m_config.model_input_image_size;
    pinfo.source_image_size = source_image_size;
    pinfo.model_input_image_size = model_input_image_size;
    pinfo.roi_in_source_image = cv::Rect(0, 0, source_image_size.width, source_image_size.height);

    // compute the roi in the model input image
    auto dst_roi_size = redoxi_works::image_utils::compute_resize_to_fit_and_keep_aspect_ratio(
        source_image_size, model_input_image_size);
    pinfo.roi_in_model_input_image = cv::Rect(0, 0, dst_roi_size.width, dst_roi_size.height);

    // resize the image to model input size, filling part of the image with 0 if necessary
    cv::Mat resized_image(model_input_image_size, input_image.type(), cv::Scalar(0));
    cv::resize(input_image, resized_image(pinfo.roi_in_model_input_image), dst_roi_size, 0, 0, m_config.interp_method);

    // convert to float and normalize to [0,1]
    if (resized_image.type() == CV_8UC1 || resized_image.type() == CV_8UC3) {
        resized_image.convertTo(resized_image, CV_32F, 1.0 / 255.0);
    } else if (resized_image.type() == CV_32FC1 || resized_image.type() == CV_32FC3) {
        throw std::runtime_error("Unsupported image type: " + std::to_string(resized_image.type()));
    }

    // convert to CHW format, by splitting the image into channels
    std::vector<cv::Mat> channels;
    cv::split(resized_image, channels);

    // handle different image formats
    std::vector<int> channel_map;
    if (image_format == "rgb") {
        channel_map = {0, 1, 2}; // RGB -> RGB, read the channels in the order of R, G, B
    } else if (image_format == "bgr") {
        channel_map = {2, 1, 0}; // BGR -> RGB, read the channels in the order of B, G, R
    } else if (image_format == "gray") {
        channel_map = {0, 0, 0}; // Grayscale, read the channels in the order of 0, 0, 0
    } else {
        throw std::runtime_error("Unsupported image format: " + image_format);
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
        float *out_ptr = output_tensor_chw + index_output * model_input_image_size.height * model_input_image_size.width;
        channel_mat.copyTo(cv::Mat(model_input_image_size, CV_32F, out_ptr));
    }

    // fill preprocess info if requested
    if (output_preprocess_info) {
        *output_preprocess_info = pinfo;
    }
}
} // namespace redoxi_works::inference::yolo8
