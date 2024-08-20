#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class PoseDetectorIn;

class PoseDetectorInDownstreamPipelineNode
{
  public:
    virtual ~PoseDetectorInDownstreamPipelineNode()
    {
    }
    std::string accept_document_action;
};

class PoseDetectorInDownstreamModelNode
{
  public:
    virtual ~PoseDetectorInDownstreamModelNode()
    {
    }
    std::string accept_detections_action;
};

class PoseDetectorInInitConfig
{
  public:
    virtual ~PoseDetectorInInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamPipelineNode = PoseDetectorInDownstreamPipelineNode;
    std::map<std::string, DownstreamPipelineNode> pipeline_downstreams;

    using DownstreamModelNode = PoseDetectorInDownstreamModelNode;
    std::map<std::string, DownstreamModelNode> model_downstreams;

    std::string process_document_action;

    void from_parameters(PoseDetectorIn *node);
};

class PoseDetectorInRuntimeConfig
{
  public:
    virtual ~PoseDetectorInRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    void from_parameters(PoseDetectorIn *node);
};
} // namespace FlowRos2Pipeline