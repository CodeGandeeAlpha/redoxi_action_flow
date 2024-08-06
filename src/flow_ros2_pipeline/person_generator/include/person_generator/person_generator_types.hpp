#pragma once
#include <map>
#include <string>

#include <psg_actions/action/process_psg_document.hpp>
#include <psg_private_msgs/msg/psg_document.hpp>


namespace FlowRos2Pipeline
{
class PersonGenerator;

class PersonGeneratorDownstreamNode
{
  public:
    virtual ~PersonGeneratorDownstreamNode()
    {
    }
    std::string accept_document_action;
    std::string accept_frame_service;
};

class PersonGeneratorInitConfig
{
  public:
    virtual ~PersonGeneratorInitConfig()
    {
    }

    // downstream nodes, mapping node name to node configurations
    using DownstreamNode = PersonGeneratorDownstreamNode;
    std::map<std::string, DownstreamNode> downstreams;

    void from_parameters(PersonGenerator *node);
};

class PersonGeneratorRuntimeConfig
{
  public:
    virtual ~PersonGeneratorRuntimeConfig()
    {
    }
    double frame_internal_ms = -1; // TODO: add more runtime configurations
    void from_parameters(PersonGenerator *node);
};
} // namespace FlowRos2Pipeline