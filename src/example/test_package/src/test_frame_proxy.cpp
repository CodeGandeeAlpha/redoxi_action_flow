#include <optional>
#include <chrono>
#include <filesystem>
#include <redoxi_public_msgs/msg/multi_device_frame.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <spdlog/spdlog.h>

namespace rdx = redoxi_works;
namespace fs = std::filesystem;
using redoxi_works::image_utils::FrameMediator;

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
    if (false) {
        spdlog::info("no conversion");
        FrameMediator frame_mediator(img);
        cv::Mat out;
        frame_mediator.to_cv_image_copy(out);
        cv::imshow("test-out", out);
        cv::waitKey(0);
    }

    // convert encoding to rgb8
    if (false) {
        spdlog::info("convert to rgb8");
        FrameMediator frame_mediator(img);
        cv::Mat out;
        frame_mediator.to_cv_image_copy(out, "rgb8");
        cv::imshow("test-out", out);
        cv::waitKey(0);
    }

    // convert to message and then back
    {
        spdlog::info("convert to msg and then back");
        FrameMediator frame_mediator(img);

        spdlog::info("original frame width={}, height={}, encoding: {}",
                     frame_mediator.get_width(), frame_mediator.get_height(),
                     frame_mediator.get_encoding());

        FrameMediator::FrameMsg_t msg;
        frame_mediator.to_frame_msg(msg);
        spdlog::info("converted frame width={}, height={}, encoding: {}",
                     msg.metadata.width, msg.metadata.height, msg.metadata.encoding);

        FrameMediator frame_mediator_msg(&msg);
        cv::Mat out;
        frame_mediator_msg.to_cv_image_copy(out);

        spdlog::info("converted back frame width={}, height={}, encoding: {}",
                     frame_mediator_msg.get_width(), frame_mediator_msg.get_height(),
                     frame_mediator_msg.get_encoding());

        cv::imshow("test-out", out);
        cv::waitKey(0);

        //! Try converting to rgb8 encoding when converting to message
        spdlog::info("convert to msg with rgb8 encoding and then back");
        FrameMediator frame_mediator2(img);
        FrameMediator::FrameMsg_t msg2;
        frame_mediator2.to_frame_msg(msg2, "rgb8");
        spdlog::info("converted frame width={}, height={}, encoding: {}",
                     msg2.metadata.width, msg2.metadata.height, msg2.metadata.encoding);

        FrameMediator frame_mediator_msg2(&msg2);
        cv::Mat out2;
        out2 = frame_mediator_msg2.to_cv_image_shared();
        cv::imshow("test-out-rgb8", out2);
        cv::waitKey(0);
    }

    return 0;
}
