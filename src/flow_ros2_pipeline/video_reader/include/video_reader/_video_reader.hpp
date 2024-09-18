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
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Error.hpp>

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

    cv::Mat orbbec_color_to_cvmat(std::shared_ptr<ob::ColorFrame> &colorFrame)
    {
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(0);
        compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
        compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
        cv::Mat colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
        return colorRawMat;
    }


    rclcpp::Logger logger;
    std::shared_ptr<vineyard::Client> v6d_client;
    std::shared_ptr<cv::VideoCapture> video_capture;

    // orbbec pipeline
    std::shared_ptr<ob::Context> ob_ctx;
    std::shared_ptr<ob::Device> net_device;
    std::shared_ptr<ob::Pipeline> ob_pipeline;
    std::shared_ptr<ob::FrameSet> current_frameset;
    std::mutex frameset_mutex;
    std::shared_ptr<ob::FormatConvertFilter> format_convert_filter;

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
};
} // namespace FlowRos2Pipeline