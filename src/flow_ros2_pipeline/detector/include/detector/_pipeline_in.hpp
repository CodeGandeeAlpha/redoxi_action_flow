#pragma once

#include <boost/thread/synchronized_value.hpp>
#include <detector/pipeline_in.hpp>
#include <memory>
#include <thread>

namespace FlowRos2Pipeline
{
class DetectorInImpl
{
  public:
    virtual ~DetectorInImpl()
    {
    }
    DetectorInImpl(DetectorIn *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;


    boost::synchronized_value<DetectorIn::Map_Document_Waiting *> sync_document_waiting_map;
    boost::synchronized_value<DetectorIn::Map_Document_Doing *> sync_document_doing_map;
    boost::synchronized_value<DetectorIn::Map_Frame_Waiting *> sync_frame_waiting_map;
    boost::synchronized_value<DetectorIn::Map_Frame_Doing *> sync_frame_doing_map;

    std::shared_ptr<std::thread> step_thread;
    bool step_running = false; // for stopping the step thread
};
} // namespace FlowRos2Pipeline