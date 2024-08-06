#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class DetectorOut;

class DetectorOutDownstreamNode
{
  public:
    virtual ~DetectorOutDownstreamNode()
    {
    }
    std::string accept_document_action;
};

class DetectorOutInitConfig
{
  public:
    virtual ~DetectorOutInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamNode = DetectorOutDownstreamNode;
    std::map<std::string, DownstreamNode> downstreams;

    std::string process_document_action;
    std::string process_detections_action;

    void from_parameters(DetectorOut *node);
};

class DetectorOutRuntimeConfig
{
  public:
    virtual ~DetectorOutRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    void from_parameters(DetectorOut *node);
};
} // namespace FlowRos2Pipeline