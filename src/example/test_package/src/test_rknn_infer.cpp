#include "redoxi_inference_rknn/RknnModelInference.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <xtensor/xarray.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xadapt.hpp>
#include <ranges>

namespace rdx_rknn = redoxi_works::inference::rknn;
namespace rdx = redoxi_works;
namespace fs = std::filesystem;
namespace common_keys = rdx::inference::common_config_keys;
namespace common_device_types = rdx::inference::common_device_types;

// fs::path model_path = "/data/code/psf_ros2_ws/tmp/models/rknn/mobilenet_v1.rknn";
fs::path model_path = "/data/code/psf_ros2_ws/tmp/models/rknn/yolov8s-pose-pthq.rknn";
// fs::path model_path = "/data/code/psf_ros2_ws/tmp/models/rknn/yolov8n-pose-fp-bs1.rknn";
fs::path image_path = "/data/code/psf_ros2_ws/data/ori_img.jpg";

template <typename DataType, typename ShapeType = std::vector<size_t>>
    requires std::ranges::range<ShapeType>
void save_tensor_to_npy(const DataType *tensor_data, const ShapeType &shape, const std::string &output_path)
{
    auto num_elements = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<size_t>());
    auto xt_data = xt::adapt(tensor_data, num_elements, xt::no_ownership(), shape);
    xt::dump_npy(output_path, xt_data);
}

void resize_and_pad_image(cv::Mat *output, cv::Rect *roi_image_in_output,
                          const cv::Mat &image, const cv::Size &target_size,
                          const cv::Scalar &padding_color = cv::Scalar(0, 0, 0))
{
    //! Calculate scaling factor to maintain aspect ratio
    double scale = std::min((double)target_size.width / image.cols,
                            (double)target_size.height / image.rows);

    //! Calculate new size after scaling
    int scaled_width = static_cast<int>(image.cols * scale);
    int scaled_height = static_cast<int>(image.rows * scale);

    //! Calculate padding
    int pad_left = (target_size.width - scaled_width) / 2;
    int pad_top = (target_size.height - scaled_height) / 2;

    //! Store the ROI if pointer provided
    if (roi_image_in_output != nullptr) {
        *roi_image_in_output = cv::Rect(pad_left, pad_top, scaled_width, scaled_height);
    }

    //! Only process image if output is provided
    if (output != nullptr) {
        //! Create temporary Mat for resized image
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(scaled_width, scaled_height));

        //! Create output Mat
        *output = cv::Mat(target_size, image.type(), padding_color);

        //! Copy resized image to padded output
        cv::Rect roi(pad_left, pad_top, scaled_width, scaled_height);
        resized.copyTo((*output)(roi));
    }
}

int main(int argc, char **argv)
{
    rdx_rknn::RknnModelInference model;

    // create init params and fill it
    spdlog::info("Creating init params");
    auto init_params = std::dynamic_pointer_cast<rdx_rknn::RknnModelConfig>(model.create_init_params());
    init_params->set_string(common_keys::ModelPath, model_path.string());
    init_params->set_string(common_keys::DeviceType, common_device_types::RKNPU);

    // open the model
    spdlog::info("Opening the model");
    auto ret_open = model.open(init_params);
    spdlog::info("Model opened with return code: {}", ret_open);

    // test random data input
    spdlog::info("Creating inference inout data");
    auto inout_data = model.create_inference_inout_data();
    auto input_port_name = model.get_input_port_infos().begin()->first;
    // spdlog::info("Getting input port data from inout_data : {}", (int64_t)inout_data.get());
    RDX_INFO_DEV(nullptr, __func__, false, "input_port_name: {}", input_port_name);
    auto input_data = inout_data->get_input_port_data(input_port_name);
    auto input_shape = input_data->get_shape();
    spdlog::info("Input shape: {}", fmt::join(input_shape, ","));

    // generate a random image
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    cv::Mat resized_image;
    cv::Rect roi_image_in_output;
    resize_and_pad_image(&resized_image, &roi_image_in_output, image, cv::Size(input_shape[2], input_shape[1]));

    // save the resized image
    cv::imwrite("/data/code/psf_ros2_ws/tmp/output/rknn-pose/resized_image.jpg", resized_image);

    spdlog::info("Setting input data");
    {
        std::vector<cv::Mat> split_channels;
        std::vector<size_t> shape_chw = {(size_t)resized_image.channels(), (size_t)resized_image.rows, (size_t)resized_image.cols};
        cv::split(resized_image, split_channels);
        xt::xarray<uint8_t> input_tensor(shape_chw);
        for (int i = 0; i < resized_image.channels(); i++) {
            auto a = xt::adapt((uint8_t *)split_channels[i].data,
                               (size_t)split_channels[i].total(),
                               xt::no_ownership(), shape_chw);
            xt::view(input_tensor, i, xt::all(), xt::all()) = a;
        }
        input_data->set_tensor_data(input_tensor.data(), input_shape);
    }

    // run inference
    spdlog::info("Running inference");
    auto ret_inference = model.do_inference(inout_data);
    spdlog::info("Inference returned with code: {}", ret_inference);

    // read output data and save to numpy
    auto all_output_ports = model.get_output_port_infos();
    for (const auto &[port_name, port_info] : all_output_ports) {
        auto output_data = inout_data->get_output_port_data(port_name);
        auto output_shape = output_data->get_shape();
        spdlog::info("Output name: {}, shape: {}", port_name, fmt::join(output_shape, ","));

        auto shape = output_data->get_shape();
        float *data = nullptr;
        output_data->get_tensor_data(&data);
        // Wrap raw data into xtensor with corresponding shape

        // Save tensor to numpy file
        std::string output_dir = "/data/code/psf_ros2_ws/tmp/output/rknn-pose";
        std::string output_path = output_dir + "/" + port_name + ".npy";

        //! Save tensor using xtensor-io
        save_tensor_to_npy(data, shape, output_path);
        spdlog::info("Saved output tensor to {}", output_path);
    }

    spdlog::info("End of test_rknn_infer");
    return 0;
}
