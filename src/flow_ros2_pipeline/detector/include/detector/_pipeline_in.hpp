#pragma once

#include <memory>
#include <vector>
#include <thread>

#include "detector/pipeline_in.hpp"

namespace FlowRos2Pipeline {
    class DetectorInImpl{
    public:
        virtual ~DetectorInImpl(){}
        DetectorInImpl(DetectorIn* node): logger(node->get_logger()){}
        rclcpp::Logger logger;

        std::shared_ptr<std::thread> step_thread;
        bool step_running = false;  // for stopping the step thread
    };
}