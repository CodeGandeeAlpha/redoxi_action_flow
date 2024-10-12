#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class PoseDetectorPipeline;

class PoseDetectorPipelineDownstreamPipelineNode
{
  public:
    virtual ~PoseDetectorPipelineDownstreamPipelineNode()
    {
    }
    // client action name
    std::string accept_document_action;
};

class PoseDetectorPipelineDownstreamModelNode
{
  public:
    virtual ~PoseDetectorPipelineDownstreamModelNode()
    {
    }
    // client action name
    std::string accept_detections_action;
};

class PoseDetectorPipelineInitConfig
{
  public:
    virtual ~PoseDetectorPipelineInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamPipelineNode = PoseDetectorPipelineDownstreamPipelineNode;
    std::map<std::string, DownstreamPipelineNode> pipeline_downstreams;

    using DownstreamModelNode = PoseDetectorPipelineDownstreamModelNode;
    std::map<std::string, DownstreamModelNode> model_downstreams;

    // action server names
    std::string process_document_action;
    std::string process_model_results_action;

    void from_parameters(PoseDetectorPipeline *node);
};

class PoseDetectorPipelineRuntimeConfig
{
  public:
    virtual ~PoseDetectorPipelineRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    bool send_goal_retry = false; // retry when send goal failed
    int buffer_size = 1;          // buffer size for sending task to downstream

    void from_parameters(PoseDetectorPipeline *node);
};
} // namespace FlowRos2Pipeline