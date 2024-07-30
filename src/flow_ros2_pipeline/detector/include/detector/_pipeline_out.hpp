#pragma once

#include <memory>
#include <thread>

#include "detector/pipeline_out.hpp"

namespace FlowRos2Pipeline {
    class DetectorOutImpl{
    public:
        virtual ~DetectorOutImpl(){}
        DetectorOutImpl(DetectorOut* node): logger(node->get_logger()){}
        rclcpp::Logger logger;

        std::shared_ptr<std::thread> step_thread;
        bool step_running = false;  // for stopping the step thread
    };
}