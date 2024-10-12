// 将pipeline in和pipeline out整合
#pragma once

#include <boost/thread/synchronized_value.hpp>
#include <detector/pipeline.hpp>
#include <memory>
#include <thread>

namespace FlowRos2Pipeline
{
class DetectorPipelineImpl
{
  public:
    virtual ~DetectorPipelineImpl()
    {
    }
    DetectorPipelineImpl(DetectorPipeline *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;


    boost::synchronized_value<DetectorPipeline::Map_Document_Waiting *> sync_document_waiting_map;
    // boost::synchronized_value<DetectorPipeline::Map_Document_Doing *> sync_document_doing_map;
    boost::synchronized_value<DetectorPipeline::Map_Frame_Waiting *> sync_frame_waiting_map;
    // boost::synchronized_value<DetectorPipeline::Map_Frame_Doing *> sync_frame_doing_map;

    boost::synchronized_value<std::map<int, DetectorPipeline::MSG_PsgDocument> *> sync_document_buffer;
    boost::synchronized_value<std::map<int, std::vector<DetectorPipeline::MSG_Detections>> *> sync_detections_buffer;

    std::shared_ptr<std::thread> step_thread;
    std::shared_ptr<std::thread> process_thread;
    bool step_running = false; // for stopping the step thread
};
} // namespace FlowRos2Pipeline