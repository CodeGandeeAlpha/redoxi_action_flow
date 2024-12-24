#include "redoxi_inference_rknn/RknnModelInference.hpp"
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

namespace rdx_rknn = redoxi_works::inference::rknn;
namespace rdx = redoxi_works;
namespace fs = std::filesystem;
namespace common_keys = rdx::inference::common_config_keys;
namespace common_device_types = rdx::inference::common_device_types;

// fs::path model_path = "/data/code/psf_ros2_ws/tmp/models/rknn/mobilenet_v1.rknn";
// fs::path model_path = "/data/code/psf_ros2_ws/tmp/models/rknn/yolov8n-pose-ptq-bs3.rknn";
fs::path model_path = "/data/code/psf_ros2_ws/tmp/models/rknn/yolov8n-pose-fp-bs3.rknn";
fs::path image_path = "/data/code/psf_ros2_ws/data/ori_img.jpg";

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
    cv::Mat random_image(input_shape[1], input_shape[2], CV_8UC3, cv::Scalar(0, 0, 0));
    cv::randu(random_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    spdlog::info("Setting input data");
    input_data->set_tensor_data(random_image.data, input_shape);

    // run inference
    spdlog::info("Running inference");
    auto ret_inference = model.do_inference(inout_data);
    spdlog::info("Inference returned with code: {}", ret_inference);

    spdlog::info("End of test_rknn_infer");
    return 0;
}
