#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class TrackerOut;

class TrackerOutDownstreamNode
{
  public:
    virtual ~TrackerOutDownstreamNode()
    {
    }
    std::string accept_document_action;
};

class TrackerOutInitConfig
{
  public:
    virtual ~TrackerOutInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamNode = TrackerOutDownstreamNode;
    std::map<std::string, DownstreamNode> downstreams;

    std::string process_document_action;
    std::string process_track_targets_action;

    void from_parameters(TrackerOut *node);
};

class TrackerOutRuntimeConfig
{
  public:
    virtual ~TrackerOutRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    void from_parameters(TrackerOut *node);
};
} // namespace FlowRos2Pipeline