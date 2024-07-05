#include "test_pg/simple_lib.hpp"
#include <RedoxiTracking/tracker/BotsortTracker.h>
#include <opencv2/opencv.hpp>
// #include "RedoxiTracking.h"


TestPSGFlow::TestPSGFlow(): rclcpp::Node("test_psgflow") {
    auto logger = rclcpp::get_logger("test_psgflow");
    RCLCPP_INFO(logger, "[TestPSGFlow] init success!");

    // 设置图像的宽度、高度和类型
    int width = 640;
    int height = 480;
    int type = CV_8UC3; // 8位无符号整数的3通道图像（RGB）

    // 创建一个指定尺寸和类型的图像
    cv::Mat image(height, width, type);

    // 使用随机值填充图像
    cv::randu(image, cv::Scalar::all(0), cv::Scalar::all(255));
    RCLCPP_INFO(logger, "[TestPSGFlow] opencv test success!");
    std::cout << image << std::endl;

    RedoxiTracking::BotsortTracker tracker;
}


TestPSGFlow::~TestPSGFlow()
{
    auto logger = rclcpp::get_logger("test_psgflow");
    RCLCPP_INFO(logger, "[TestPSGFlow] stop!");
}