#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class DetectorPipeline;

class DetectorPipelineDownstreamPipelineNode
{
  public:
    virtual ~DetectorPipelineDownstreamPipelineNode()
    {
    }
    // client action name
    std::string accept_document_action;
};

class DetectorPipelineDownstreamModelNode
{
  public:
    virtual ~DetectorPipelineDownstreamModelNode()
    {
    }
    // client action name
    std::string accept_frame_action;
};

class DetectorPipelineInitConfig
{
  public:
    virtual ~DetectorPipelineInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamPipelineNode = DetectorPipelineDownstreamPipelineNode;
    std::map<std::string, DownstreamPipelineNode> pipeline_downstreams;

    using DownstreamModelNode = DetectorPipelineDownstreamModelNode;
    std::map<std::string, DownstreamModelNode> model_downstreams;

    // action server names
    std::string process_document_action;
    std::string process_model_results_action;

    void from_parameters(DetectorPipeline *node);
};

class DetectorPipelineRuntimeConfig
{
  public:
    virtual ~DetectorPipelineRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    bool send_goal_retry = false; // retry when send goal failed
    int buffer_size = 1;          // buffer size for sending task to downstream

    void from_parameters(DetectorPipeline *node);
};
} // namespace FlowRos2Pipeline