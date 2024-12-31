#include <test_package/_pch.hpp>

#include <redoxi_inference/redoxi_inference.hpp>
#include <redoxi_basic_cpp/logging/ros_logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

namespace rdx_infer = redoxi_works::inference;
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
    rclcpp::init(argc, argv);

    pluginlib::ClassLoader<rdx_infer::RedoxiModelInference> loader("redoxi_inference", "redoxi_works::inference::RedoxiModelInference");
    try {
        spdlog::info("Creating model inference instance");
        auto model = loader.createSharedInstance("redoxi_works::inference::rknn::RknnModelInference");

        spdlog::info("Creating init params");
        auto init_params = model->create_init_params();
        init_params->set_string(common_keys::ModelPath, model_path.string());
        // init_params->set_string(common_keys::DeviceType, common_device_types::RKNPU);

        spdlog::info("Opening model");
        auto ret_open = model->open(init_params);
        spdlog::info("Model opened with return code: {}", ret_open);

        spdlog::info("Creating inference inout data");
        auto inout_data = model->create_inference_inout_data();
        auto input_port_name = model->get_input_port_infos().begin()->first;
        spdlog::info("Input port name: {}", input_port_name);

        auto input_data = inout_data->get_input_port_data(input_port_name);
        auto input_shape = input_data->get_shape();
        spdlog::info("Input shape: {}", fmt::join(input_shape, ","));

        cv::Mat random_image(input_shape[1], input_shape[2], CV_8UC3, cv::Scalar(0, 0, 0));
        cv::randu(random_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        spdlog::info("Setting input data");
        input_data->set_tensor_data(random_image.data, input_shape);

        spdlog::info("Running inference");
        auto ret_inference = model->do_inference(inout_data);
        spdlog::info("Inference returned with code: {}", ret_inference);

        spdlog::info("End of test_rknn_plugin");
    } catch (const std::exception &e) {
        spdlog::error("Error loading plugin: {}", e.what());
    }
    return 0;
}