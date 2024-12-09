#include <optional>
#include <chrono>
#include <filesystem>
#include <redoxi_public_msgs/msg/multi_device_frame.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <test_package/FrameMediator.hpp>
#include <spdlog/spdlog.h>

namespace rdx = redoxi_works;
namespace fs = std::filesystem;

#ifndef TEST_DATA_DIR
#    define TEST_DATA_DIR "/home/hard/volume/workspace/code/psf_ros2_ws/src/example/test_package/data"
#endif

#ifndef DISPLAY_ENV_VAR
#    define DISPLAY_ENV_VAR ""
#endif

fs::path TestDataDir(TEST_DATA_DIR);
fs::path TestImagePath = TestDataDir / "ori_img.jpg";

int main(int argc, char **argv)
{
    cv::Mat img = cv::imread(TestImagePath.string());

    // convert encoding to rgb8
    {
        spdlog::info("no conversion");
        rdx::FrameMediator frame_mediator(img);
        cv::Mat out;
        frame_mediator.to_cv_image_copy(out);
        cv::imshow("test-out", out);
        cv::waitKey(0);
    }

    // convert encoding to rgb8
    {
        spdlog::info("convert to rgb8");
        rdx::FrameMediator frame_mediator(img);
        cv::Mat out;
        frame_mediator.to_cv_image_copy(out, "rgb8");
        cv::imshow("test-out", out);
        cv::waitKey(0);
    }
    return 0;
}
