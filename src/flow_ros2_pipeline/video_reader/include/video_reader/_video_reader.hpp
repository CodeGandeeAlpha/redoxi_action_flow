#pragma once

#include <opencv2/opencv.hpp>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/timer.hpp>

#include "common/util/uuid.h"
#include "vineyard/client/client.h"
#include "vineyard/client/ds/object_meta.h"
#include "video_reader/video_reader.hpp"

namespace FlowRos2Pipeline {
    class OpencvVideoReaderImpl{
    public:
        virtual ~OpencvVideoReaderImpl(){}
        OpencvVideoReaderImpl(OpencvVideoReader* node): logger(node->get_logger()){
            ros_clock = rclcpp::Clock(RCL_ROS_TIME);
        }
        rclcpp::Logger logger;
        std::shared_ptr<vineyard::Client> v6d_client;
        std::shared_ptr<cv::VideoCapture> video_capture;
        rclcpp::TimerBase::SharedPtr step_timer;
        rclcpp::TimerBase::SharedPtr frame_timer;
        bool ready_to_read_next_frame = true;
        cv::Mat src_frame;  // last read frame, avoid creating cv::Mat object every time
        cv::Mat resized_frame; // resized frame

    private:
        // the time we read the last frame
        rclcpp::Clock ros_clock;
        rclcpp::Time time_reading_last_frame;
    };
}