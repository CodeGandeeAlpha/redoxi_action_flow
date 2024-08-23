#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class TrackerIn;

class TrackerInDownstreamPipelineNode
{
  public:
    virtual ~TrackerInDownstreamPipelineNode()
    {
    }
    std::string accept_document_action;
};

class TrackerInDownstreamModelNode
{
  public:
    virtual ~TrackerInDownstreamModelNode()
    {
    }
    std::string accept_detections_action;
};

class TrackerInInitConfig
{
  public:
    virtual ~TrackerInInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamPipelineNode = TrackerInDownstreamPipelineNode;
    std::map<std::string, DownstreamPipelineNode> pipeline_downstreams;

    using DownstreamModelNode = TrackerInDownstreamModelNode;
    std::map<std::string, DownstreamModelNode> model_downstreams;

    std::string process_document_action;

    void from_parameters(TrackerIn *node);
};

class TrackerInRuntimeConfig
{
  public:
    virtual ~TrackerInRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    void from_parameters(TrackerIn *node);
};
} // namespace FlowRos2Pipeline