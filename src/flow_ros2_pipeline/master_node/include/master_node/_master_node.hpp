#pragma once

#include <master_node/master_node.hpp>
#include <rclcpp/timer.hpp>

namespace FlowRos2Pipeline {
    class MasterNodeImpl {
    public:
        MasterNodeImpl(MasterNode* node) : logger(node->get_logger()) {}
        virtual ~MasterNodeImpl() {}
        rclcpp::Logger logger;
        rclcpp::TimerBase::SharedPtr timer;
    };
}