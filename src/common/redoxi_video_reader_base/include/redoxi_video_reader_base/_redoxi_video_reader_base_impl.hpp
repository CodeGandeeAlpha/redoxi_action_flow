#pragma once

#include <redoxi_video_reader_base/redoxi_video_reader_base.hpp>
#include <redoxi_common_cpp/redoxi_v6d.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>

#include <tbb/tbb.h>

namespace redoxi_works
{

class RedoxiVideoReaderImpl
{
  public:
    virtual ~RedoxiVideoReaderImpl()
    {
    }
    RedoxiVideoReaderImpl(RedoxiVideoReaderBase *node)
        : logger(node->get_logger())
    {
    }

    rclcpp::Logger logger;
    std::shared_ptr<vineyard::Client> v6d_client;
    std::shared_ptr<cv::VideoCapture> video_capture;

    rclcpp::TimerBase::SharedPtr step_timer;
    rclcpp::TimerBase::SharedPtr frame_timer;
    bool ready_to_read_next_frame = true;
    bool read_frame_ok = false;
    bool is_video_end = false;
    cv::Mat src_frame;     // last read frame, avoid creating cv::Mat object every time
    cv::Mat resized_frame; // resized frame

    // flags to convert async call to sync call
    std::vector<bool> frame_sent_flags;
    std::shared_ptr<std::thread> step_thread;
    bool step_running = false; // for stopping the step thread

    // token for reading a new frame every x-milliseconds
    std::shared_ptr<RosTimeToken_ms> read_frame_token;
};
} // namespace redoxi_works
