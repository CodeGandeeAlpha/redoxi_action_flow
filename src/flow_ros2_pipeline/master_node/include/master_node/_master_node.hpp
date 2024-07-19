#pragma once

#include <master_node/master_node.hpp>
#include <memory>
#include <rclcpp/timer.hpp>
#include <thread>

namespace FlowRos2Pipeline {
    class MasterNodeImpl {
    public:
        MasterNodeImpl(MasterNode* node) : logger(node->get_logger()) {}
        virtual ~MasterNodeImpl() {}
        rclcpp::Logger logger;
        rclcpp::TimerBase::SharedPtr timer;

        std::shared_ptr<std::thread> step_thread;
        bool step_running = false;
    };
}