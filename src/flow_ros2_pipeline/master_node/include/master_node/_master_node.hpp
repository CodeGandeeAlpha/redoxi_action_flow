#pragma once

#include <boost/thread/synchronized_value.hpp>
#include <memory>
#include <thread>

#include <rclcpp/timer.hpp>

#include <master_node/master_node.hpp>

namespace FlowRos2Pipeline
{
class MasterNodeImpl
{
  public:
    MasterNodeImpl(MasterNode *node)
        : logger(node->get_logger())
    {
    }
    virtual ~MasterNodeImpl()
    {
    }
    rclcpp::Logger logger;

    boost::synchronized_value<MasterNode::Map_Document_Waiting *> sync_document_waiting_map;
    // boost::synchronized_value<MasterNode::Map_Document_Doing *> sync_document_doing_map;

    boost::synchronized_value<std::map<int, MasterNode::MSG_Frame> *> sync_frame_buffer;

    std::shared_ptr<std::thread> step_thread;
    bool step_running = false;

    std::atomic_bool accepted_flag = false;
};
} // namespace FlowRos2Pipeline