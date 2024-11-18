#include <redoxi_dnn_models/Yolo8Pose.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
// #include <redoxi_common_cpp/ros_utils/common.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <xtensor/xtensor.hpp>
#include <xtensor/xnpy.hpp>
#ifndef TEST_MODEL_DIR
#    define TEST_MODEL_DIR "./models"
#endif

#ifndef TEST_DATA_DIR
#    define TEST_DATA_DIR "./data"
#endif

#ifndef TEST_OUTPUT_DIR
#    define TEST_OUTPUT_DIR "./tmp/output"
#endif

namespace fs = std::filesystem;
namespace rdx_inf = redoxi_works::inference;
namespace cmkeys = rdx_inf::common_config_keys;
namespace cmdev = rdx_inf::common_device_types;
const auto fn_model = fs::path(TEST_MODEL_DIR) / "yolov8n-pose-dynbatch.onnx";
const auto fn_image = fs::path(TEST_DATA_DIR) / "pose-sample.jpg";
const auto fn_pose_output = fs::path(TEST_OUTPUT_DIR) / "pose-output.jpg";
const auto fn_tensor_output = fs::path(TEST_OUTPUT_DIR) / "test-tensor.npy";

int _main(int argc, char **argv)
{
    //! Load image from file
    cv::Mat image = cv::imread(fn_image.string());

    // create a tensor from the image
    xt::xtensor<uint8_t, 3> tensor({(size_t)image.rows, (size_t)image.cols, 3});
    std::memcpy(tensor.data(), image.data, tensor.size() * sizeof(uint8_t));

    // save the tensor to a file
    spdlog::info("Saving tensor to {}", fn_tensor_output.string());
    xt::dump_npy(fn_tensor_output.string(), tensor);
    return 0;
}

int main(int argc, char **argv)
{
    // if output directory does not exist, create it
    if (!fs::exists(TEST_OUTPUT_DIR)) {
        fs::create_directories(TEST_OUTPUT_DIR);
    }

    rdx_inf::Yolo8Pose yolo_model;

    // load model
    {
        spdlog::info("Loading model from {}", fn_model.string());
        auto params = yolo_model.create_init_params();
        params->set_string(cmkeys::ModelPath, fn_model.string());
        params->set_string(cmkeys::DeviceType, cmdev::CPU);
        yolo_model.open(params);
        spdlog::info("Model loaded successfully");
    }

    // load an image and do inference
    {
        cv::Size model_input_size(640, 640);
        cv::Mat image = cv::imread(fn_image.string());
        if (image.empty()) {
            spdlog::error("Failed to load image from {}", fn_image.string());
            return -1;
        }

        // convert to RGB
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

        // scale image to 640x640, keeping aspect ratio
        // auto new_size = redoxi_works::image_utils::compute_resize_to_fit_and_keep_aspect_ratio(image.size(), model_input_size);
        // cv::Mat resized_image(model_input_size, image.type());
        // cv::Rect roi(0, 0, new_size.width, new_size.height);
        // cv::resize(image, resized_image(roi), new_size);
        auto resized_image = image;

        // create inference inout data
        spdlog::info("Creating inference inout data");
        auto inout_data = yolo_model.create_inference_inout_data();

        spdlog::info("Setting input images");
        yolo_model.set_input_images(inout_data, {resized_image});

        spdlog::info("Running inference");
        yolo_model.do_inference(inout_data);

        spdlog::info("Getting output detections");
        auto detections = yolo_model.get_output_detections(inout_data);
        spdlog::info("Output detections: {}", detections[0].objects.size());

        // print detections
        // for (size_t i = 0; i < detections[0].objects.size(); ++i) {
        //     const auto &det = detections[0].objects[i];
        //     spdlog::info("Detection index = {}", i);
        //     spdlog::info("  BBox: {} {} {} {}", det.xywh[0], det.xywh[1], det.xywh[2], det.xywh[3]);
        //     spdlog::info("  Score: {}", det.score);
        // }

        // draw detections
        cv::Mat vis_image = resized_image.clone();
        for (const auto &det : detections[0].objects) {
            cv::rectangle(vis_image,
                          cv::Rect(det.xywh[0], det.xywh[1], det.xywh[2], det.xywh[3]),
                          cv::Scalar(0, 0, 255), 2);
        }
        cv::imwrite(fn_pose_output.string(), vis_image);
        // cv::imshow("Detections", vis_image);
        // cv::waitKey(0);
    }

    return 0;
}
