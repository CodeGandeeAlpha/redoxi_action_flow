#pragma once
#include <map>
#include <psg_common/psg_common.hpp>
#include <string>

namespace FlowRos2Pipeline
{
class PersonGenerator;

class PersonGeneratorDownstreamPipelineNode
{
  public:
    virtual ~PersonGeneratorDownstreamPipelineNode()
    {
    }
    std::string accept_document_action;
};

class PersonGeneratorInitConfig
{
  public:
    virtual ~PersonGeneratorInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamPipelineNode = PersonGeneratorDownstreamPipelineNode;
    std::map<std::string, DownstreamPipelineNode> pipeline_downstreams;

    std::string process_document_action;

    void from_parameters(PersonGenerator *node);
};

class PersonGeneratorRuntimeConfig
{
  public:
    virtual ~PersonGeneratorRuntimeConfig()
    {
    }

    // step frequency
    double step_interval_ms = DefaultNodeStepIntervalMs;

    double timeout_ms_send_to_downstream = DefaultTimeoutMs;

    bool send_goal_retry = false; // retry when send goal failed
    int buffer_size = 1;          // buffer size for sending task to downstreams

    void from_parameters(PersonGenerator *node);
};
} // namespace FlowRos2Pipeline