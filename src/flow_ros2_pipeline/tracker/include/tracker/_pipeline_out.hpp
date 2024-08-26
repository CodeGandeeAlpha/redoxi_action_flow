#pragma once

#include <boost/thread/synchronized_value.hpp>
#include <memory>
#include <thread>

#include <RedoxiTrack/RedoxiTrack.h>
#include <tracker/pipeline_out.hpp>

namespace FlowRos2Pipeline
{
class TrackerOutImpl
{
  public:
    virtual ~TrackerOutImpl()
    {
    }
    TrackerOutImpl(TrackerOut *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;

    boost::synchronized_value<TrackerOut::Map_Document_Waiting *> sync_document_waiting_map;
    boost::synchronized_value<TrackerOut::Map_Document_Doing *> sync_document_doing_map;

    boost::synchronized_value<std::map<int, TrackerOut::MSG_PsgDocument> *> sync_document_buffer;
    boost::synchronized_value<std::map<int, TrackerOut::MSG_TrackTargets> *> sync_track_targets_buffer;
    boost::synchronized_value<std::map<TrackerOut::MSG_UUID, TrackerOut::MSG_Person> *> sync_person_buffer;

    std::shared_ptr<std::thread> step_thread;
    bool step_running = false; // for stopping the step thread
};
} // namespace FlowRos2Pipeline