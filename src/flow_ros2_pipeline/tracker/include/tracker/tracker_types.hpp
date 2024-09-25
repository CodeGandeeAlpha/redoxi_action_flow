#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class Tracker;

namespace TrackerTypes // FIXME: remove this namespace
{
const int DEEPSORT = 0;
const int BOTSORT = 1;
}; // namespace TrackerTypes

class TrackerDownstreamPipelineNode
{
  public:
    virtual ~TrackerDownstreamPipelineNode()
    {
    }
    std::string accept_track_targets_action;
};

class TrackerInitConfig
{
  public:
    virtual ~TrackerInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamPipelineNode = TrackerDownstreamPipelineNode;
    std::map<std::string, DownstreamPipelineNode> pipeline_downstreams;

    std::string process_detections_action;
    int tracker_type = TrackerTypes::DEEPSORT;

    void from_parameters(Tracker *node);
};

class TrackerRuntimeConfig
{
  public:
    virtual ~TrackerRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    bool send_goal_retry = false; // retry when send goal failed
    int buffer_size = 1;          // buffer size for sending task to downstream

    void from_parameters(Tracker *node);
};
} // namespace FlowRos2Pipeline