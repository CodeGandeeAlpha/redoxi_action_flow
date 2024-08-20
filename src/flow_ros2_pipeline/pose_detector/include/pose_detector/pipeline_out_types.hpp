#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class PoseDetectorOut;

class PoseDetectorOutDownstreamNode
{
  public:
    virtual ~PoseDetectorOutDownstreamNode()
    {
    }
    std::string accept_document_action;
};

class PoseDetectorOutInitConfig
{
  public:
    virtual ~PoseDetectorOutInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamNode = PoseDetectorOutDownstreamNode;
    std::map<std::string, DownstreamNode> downstreams;

    std::string process_document_action;
    std::string process_bodyposes_action;

    void from_parameters(PoseDetectorOut *node);
};

class PoseDetectorOutRuntimeConfig
{
  public:
    virtual ~PoseDetectorOutRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    void from_parameters(PoseDetectorOut *node);
};
} // namespace FlowRos2Pipeline