#pragma once

#include <boost/thread/synchronized_value.hpp>
#include <memory>
#include <pose_detector/pipeline.hpp>
#include <thread>

namespace FlowRos2Pipeline
{
class PoseDetectorPipelineImpl
{
  public:
    virtual ~PoseDetectorPipelineImpl()
    {
    }
    PoseDetectorPipelineImpl(PoseDetectorPipeline *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;


    boost::synchronized_value<PoseDetectorPipeline::Map_Document_Waiting *> sync_document_waiting_map;
    // boost::synchronized_value<PoseDetectorPipeline::Map_Document_Doing *> sync_document_doing_map;
    boost::synchronized_value<PoseDetectorPipeline::Map_Detections_Waiting *> sync_detections_waiting_map;
    // boost::synchronized_value<PoseDetectorPipeline::Map_Detections_Doing *> sync_detections_doing_map;

    boost::synchronized_value<std::map<int, PoseDetectorPipeline::MSG_PsgDocument> *> sync_document_buffer;
    boost::synchronized_value<std::map<int, std::vector<PoseDetectorPipeline::MSG_Bodyposes>> *> sync_bodyposes_buffer;

    std::shared_ptr<std::thread> step_thread;
    std::shared_ptr<std::thread> process_thread;
    bool step_running = false; // for stopping the step thread
};
} // namespace FlowRos2Pipeline