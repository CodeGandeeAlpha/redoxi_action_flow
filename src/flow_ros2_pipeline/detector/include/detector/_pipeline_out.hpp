#pragma once

#include <boost/thread/synchronized_value.hpp>
#include <memory>
#include <thread>

#include "detector/pipeline_out.hpp"

namespace FlowRos2Pipeline
{
class DetectorOutImpl
{
  public:
    virtual ~DetectorOutImpl()
    {
    }
    DetectorOutImpl(DetectorOut *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;

    boost::synchronized_value<DetectorOut::Map_Document_Waiting *> sync_document_waiting_map;
    boost::synchronized_value<DetectorOut::Map_Document_Doing *> sync_document_doing_map;

    boost::synchronized_value<std::map<int, DetectorOut::MSG_PsgDocument> *> sync_document_buffer;
    boost::synchronized_value<std::map<int, DetectorOut::MSG_Detections> *> sync_detections_buffer;

    std::shared_ptr<std::thread> step_thread;
    bool step_running = false; // for stopping the step thread
};
} // namespace FlowRos2Pipeline