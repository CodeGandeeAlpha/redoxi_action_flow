#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/timer.hpp>
#include <thread>
#include <vector>

#include "video_reader/video_reader.hpp"
#include "vineyard/client/client.h"
#include "vineyard/client/ds/object_meta.h"

namespace FlowRos2Pipeline
{
class OpencvVideoReaderImpl
{
  public:
    virtual ~OpencvVideoReaderImpl()
    {
    }
    OpencvVideoReaderImpl(OpencvVideoReader *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;
    std::shared_ptr<vineyard::Client> v6d_client;
    std::shared_ptr<cv::VideoCapture> video_capture;
    rclcpp::TimerBase::SharedPtr step_timer;
    rclcpp::TimerBase::SharedPtr frame_timer;
    bool ready_to_read_next_frame = true;
    bool is_video_end = false;
    cv::Mat src_frame;     // last read frame, avoid creating cv::Mat object every time
    cv::Mat resized_frame; // resized frame

    // flags to convert async call to sync call
    std::vector<bool> frame_sent_flags;
    std::shared_ptr<std::thread> step_thread;
    bool step_running = false; // for stopping the step thread
};
} // namespace FlowRos2Pipeline