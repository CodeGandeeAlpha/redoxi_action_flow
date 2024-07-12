#pragma once

#include "video_reader/video_reader.hpp"
#include "vineyard/client/client.h"
#include "vineyard/client/client.h"
#include "vineyard/client/ds/object_meta.h"
#include <opencv2/opencv.hpp>

namespace FlowRos2Pipeline {
    class OpencvVideoReaderImpl{
    public:
        virtual ~OpencvVideoReaderImpl(){}
        OpencvVideoReaderImpl(OpencvVideoReader* node): logger(node->get_logger()){}
        rclcpp::Logger logger;
        std::shared_ptr<vineyard::Client> v6d_client;
        std::shared_ptr<cv::VideoCapture> video_capture;
        rclcpp::TimerBase::SharedPtr timer;
    };
}