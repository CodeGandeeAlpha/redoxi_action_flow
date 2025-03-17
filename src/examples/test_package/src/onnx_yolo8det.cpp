#include <test_package/_pch.hpp>

#include <redoxi_dnn_models/yolo8/Yolo8DetectionModel.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <sensor_msgs/image_encodings.hpp>
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
const auto fn_model = fs::path(TEST_MODEL_DIR) / "head-yolo.onnx";
const auto fn_image = fs::path(TEST_DATA_DIR) / "pose-sample.jpg";
const auto fn_pose_output = fs::path(TEST_OUTPUT_DIR) / "detection-out.jpg";
const auto fn_tensor_output = fs::path(TEST_OUTPUT_DIR) / "test-tensor.npy";

int main(int argc, char **argv)
{
    // if output directory does not exist, create it
    if (!fs::exists(TEST_OUTPUT_DIR)) {
        fs::create_directories(TEST_OUTPUT_DIR);
    }

    rdx_inf::yolo8::Yolo8DetectionModel yolo_model;

    // load model
    {
        spdlog::info("Loading model from {}", fn_model.string());
        auto params = yolo_model.create_init_params();
        params->set_string(cmkeys::ModelPath, fn_model.string());
        params->set_string(cmkeys::DeviceType, cmdev::CUDA);
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
        // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

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
        yolo_model.set_input_images(inout_data, {resized_image}, sensor_msgs::image_encodings::BGR8);

        spdlog::info("Running inference");
        yolo_model.do_inference(inout_data);

        spdlog::info("Getting output detections");
        redoxi_works::inference::yolo8::PostprocessorConfig config;
        config.conf_threshold = 0.25;
        config.iou_threshold = 0.45;

        // just test class selection, this is optional
        config.selected_classes = {{1, "head"}, {0, "body"}};

        // run detections
        auto detections = yolo_model.get_output_detections(inout_data, config);


        spdlog::info("Output detections: {}", detections[0].objects.size());
        // draw detections
        cv::Mat vis_image = resized_image.clone();
        for (const auto &det : detections[0].objects) {
            // Generate color based on class id
            cv::Scalar color((det.class_id * 100) % 255,
                             (det.class_id * 50) % 255,
                             (det.class_id * 150) % 255);

            // Draw bounding box
            cv::rectangle(vis_image,
                          cv::Rect(det.xywh[0], det.xywh[1], det.xywh[2], det.xywh[3]),
                          color, 2);

            // Add class id and name to the image
            cv::putText(vis_image,
                        std::to_string(det.class_id) + ":" + det.class_name,
                        cv::Point(det.xywh[0], det.xywh[1] - 5),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        2);
        }
        spdlog::info("Saving output image to {}", fn_pose_output.string());
        cv::imwrite(fn_pose_output.string(), vis_image);
    }

    // cv::imshow("Detections", vis_image);
    // cv::waitKey(0);
    return 0;
}