#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class PSGCount;

class PSGCountDownstreamPipelineNode
{
  public:
    virtual ~PSGCountDownstreamPipelineNode()
    {
    }
    std::string accept_document_action;
};

class PSGCountInitConfig
{
  public:
    virtual ~PSGCountInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamPipelineNode = PSGCountDownstreamPipelineNode;
    std::map<std::string, DownstreamPipelineNode> pipeline_downstreams;

    std::string process_document_action;
    std::string passengerflow_config_path;

    void from_parameters(PSGCount *node);
};

class PSGCountRuntimeConfig
{
  public:
    virtual ~PSGCountRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    bool send_goal_retry = false; // retry when send goal failed
    int buffer_size = 1;          // buffer size for sending task to downstreams

    void from_parameters(PSGCount *node);
};
} // namespace FlowRos2Pipeline