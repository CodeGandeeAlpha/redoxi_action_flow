#pragma once

#include <boost/thread/synchronized_value.hpp>
#include <memory>
#include <pose_detector/pipeline_in.hpp>
#include <thread>

namespace FlowRos2Pipeline
{
class PoseDetectorInImpl
{
  public:
    virtual ~PoseDetectorInImpl()
    {
    }
    PoseDetectorInImpl(PoseDetectorIn *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;


    boost::synchronized_value<PoseDetectorIn::Map_Document_Waiting *> sync_document_waiting_map;
    boost::synchronized_value<PoseDetectorIn::Map_Document_Doing *> sync_document_doing_map;
    boost::synchronized_value<PoseDetectorIn::Map_Detections_Waiting *> sync_detections_waiting_map;
    boost::synchronized_value<PoseDetectorIn::Map_Detections_Doing *> sync_detections_doing_map;

    std::shared_ptr<std::thread> step_thread;
    bool step_running = false; // for stopping the step thread
};
} // namespace FlowRos2Pipeline