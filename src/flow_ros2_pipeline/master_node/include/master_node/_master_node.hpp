#pragma once

#include <master_node/master_node.hpp>

namespace FlowRos2Pipeline {
    class MasterNodeImpl {
    public:
        virtual ~MasterNodeImpl() {}
        MasterNodeImpl(MasterNode* node) : logger(node->get_logger()) {}
        rclcpp::Logger logger;
    };
}