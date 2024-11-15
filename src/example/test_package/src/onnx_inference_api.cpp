#include <redoxi_inference/redoxi_inference.hpp>
#include <pluginlib/class_loader.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>

#ifndef TEST_MODEL_DIR
#    define TEST_MODEL_DIR "./models"
#endif

namespace fs = std::filesystem;

namespace rdx_inf = redoxi_works::inference;
const auto fn_model = fs::path(TEST_MODEL_DIR) / "yolov8n-pose-dynbatch.onnx";

const char *EpCPU = "CPUExecutionProvider";
const char *EpCUDA = "CUDAExecutionProvider";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    spdlog::info("Loading onnx inference");
    pluginlib::ClassLoader<rdx_inf::RedoxiModelInference> loader("redoxi_inference", "redoxi_works::inference::RedoxiModelInference");

    spdlog::info("Creating onnx inference instance");
    std::shared_ptr<rdx_inf::RedoxiModelInference> inference = loader.createSharedInstance("redoxi_works::inference::onnx::OnnxModelInference");

    // set model path, and open model
    {
        auto params = inference->create_init_params();
        auto all_keys = params->get_all_keys();
        spdlog::info("acceptable params:");
        for (const auto &key : all_keys) {
            spdlog::info("key: name={}, description={}", key.name, key.description);
        }
        params->set_string("model_path", fn_model.string());
        params->set_string("execution_provider", EpCUDA);

        spdlog::info("open model");
        inference->open(params);
        spdlog::info("model opened");

        // print all port info
        auto input_ports = inference->get_input_port_infos();
        auto output_ports = inference->get_output_port_infos();

        for (const auto &port : input_ports) {
            auto port_info = port.second;
            spdlog::info("{}", port_info->to_description());
        }

        for (const auto &port : output_ports) {
            auto port_info = port.second;
            spdlog::info("{}", port_info->to_description());
        }
    }

    // infer with random data
    {
        auto inout_data = inference->create_inference_inout_data();
        double image_mean = 0.0;
        {
            auto inp_images = inout_data->get_input_port_data("images");
            auto inp_images_info = inp_images->get_port_info();
            auto shape = inp_images_info->get_shape();
            auto dtype_str = inp_images_info->get_dtype_str();

            // get the shape of the input images
            int64_t batch_size = 0;
            int64_t channels = 0;
            int64_t height = 0;
            int64_t width = 0;

            if (shape.size() == 4) {
                batch_size = shape[0];
                channels = shape[1];
                height = shape[2];
                width = shape[3];
                spdlog::info("Shape: batch_size={}, channels={}, height={}, width={}", batch_size, channels, height, width);
            } else {
                spdlog::warn("Unexpected shape size: {}", shape.size());
            }

            // if batch size is not specified, set it to 4
            batch_size = batch_size > 0 ? batch_size : 4;
            std::vector<int64_t> real_shape(shape.begin(), shape.end());
            real_shape[0] = batch_size;

            // create random images
            std::vector<float> image_data;
            if (dtype_str == "float32") {
                image_data.resize(batch_size * channels * height * width);
                cv::randu(image_data, 0.0f, 1.0f); // Random values between 0 and 1
            } else if (dtype_str == "uint8") {
                image_data.resize(batch_size * channels * height * width);
                std::vector<uint8_t> image_data_uint8(image_data.begin(), image_data.end());
                cv::randu(image_data_uint8, 0, 256); // Random values between 0 and 255
                image_data.assign(image_data_uint8.begin(), image_data_uint8.end());
            } else {
                spdlog::warn("Unsupported dtype: {}", dtype_str);
            }

            if (!image_data.empty()) {
                double sum = std::accumulate(image_data.begin(), image_data.end(), 0.0);
                image_mean = sum / image_data.size();
                spdlog::info("Computed mean of image data: {}, num_elements={}", image_mean, image_data.size());
            } else {
                spdlog::warn("Image data is empty, cannot compute mean.");
            }

            // set the images to the input port
            spdlog::info("Setting tensor data to the input port");
            inp_images->set_tensor_data(image_data.data(), real_shape);
            spdlog::info("Tensor data set to the input port");
        }
        {
            spdlog::info("Testing input data read back");
            // we should be able to get back the input data, even if the original data is destroyed
            auto inp_images_data = inout_data->get_input_port_data("images");
            auto port_info = inp_images_data->get_port_info();
            auto model_shape = port_info->get_shape();
            auto real_shape = inp_images_data->get_shape();
            auto num_elements = std::accumulate(real_shape.begin(), real_shape.end(), 1, std::multiplies<int64_t>());
            spdlog::info("Number of elements: {}", num_elements);
            auto dtype_str = port_info->get_dtype_str();
            float *pixel_data = nullptr;
            inp_images_data->get_tensor_data(&pixel_data);

            double computed_mean = std::accumulate(pixel_data, pixel_data + num_elements, 0.0) / num_elements;
            spdlog::info("Computed mean of input data= {}, original mean={}", computed_mean, image_mean);
            if (std::abs(computed_mean - image_mean) > 10 * std::numeric_limits<double>::epsilon()) {
                spdlog::error("Computed mean of input data does not match the original mean");
            } else {
                spdlog::info("OK: Computed mean of input data matches the original mean");
            }
            spdlog::info("Model shape: {}", fmt::join(model_shape.begin(), model_shape.end(), ", "));
            spdlog::info("Real shape: {}", fmt::join(real_shape.begin(), real_shape.end(), ", "));
            spdlog::info("Input data dtype: {}", dtype_str);
        }
        {
            for (int ith_run = 0; ith_run < 10; ++ith_run) {
                spdlog::info("Testing inference, run {}/{}", ith_run + 1, 10);

                auto inp_images = inout_data->get_input_port_data("images");

                // Randomize the input data
                {
                    // TODO: why the output does not change, even if the input is changed?
                    // BUG: the output does not change, why?
                    auto inp_images_data = inout_data->get_input_port_data("images");
                    auto port_info = inp_images_data->get_port_info();
                    auto real_shape = inp_images_data->get_shape();
                    auto num_elements = std::accumulate(real_shape.begin(), real_shape.end(), 1, std::multiplies<int64_t>());
                    float *pixel_data = nullptr;
                    inp_images_data->get_tensor_data(&pixel_data);

                    cv::Mat tmp(1, num_elements, CV_32FC1, pixel_data);
                    cv::randu(tmp, 0.0f, 1.0f);
                }

                spdlog::info("Input data randomized");

                spdlog::info("Start inference");
                inference->do_inference(inout_data);
                spdlog::info("Inference done");

                // get the output data
                auto output_data = inout_data->get_output_port_data("output0");
                auto output_info = output_data->get_port_info();
                spdlog::info("output desc: {}", output_info->to_description());

                auto real_shape = output_data->get_shape();
                spdlog::info("output real shape: {}", fmt::join(real_shape.begin(), real_shape.end(), ", "));

                auto num_elements = std::accumulate(real_shape.begin(), real_shape.end(), 1, std::multiplies<int64_t>());
                spdlog::info("Output data number of elements: {}", num_elements);

                float *output_data_ptr = nullptr;
                output_data->get_tensor_data(&output_data_ptr);

                double computed_mean = std::accumulate(output_data_ptr, output_data_ptr + num_elements, 0.0) / num_elements;
                spdlog::info("Computed mean of output data= {}", computed_mean);

                spdlog::info("Output data retrieved");
            }
        }
        // {
        //     auto out_poses = inout_data->get_output_port_data("poses");
        //     auto out_poses_info = out_poses->get_port_info();
        //     spdlog::info("{}", out_poses_info->to_description());
        // }
    }

    spdlog::info("done");
    rclcpp::shutdown();
    return 0;
}
