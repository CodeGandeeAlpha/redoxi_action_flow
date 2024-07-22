#pragma once
#include <map>
#include <string>

#include <psg_private_msgs/msg/psg_document.hpp>


namespace FlowRos2Pipeline {
    class MasterNode;

    class MasterNodeDownstreamNode{
    public:
        virtual ~MasterNodeDownstreamNode(){}
        std::string accept_document_action;
    };

    class MasterNodeInitConfig {
    public:
        virtual ~MasterNodeInitConfig(){}

        //FIXME: support multiple downstreams
        std::string downstream_action_name;
        using DownstreamNode = MasterNodeDownstreamNode;
        std::map<std::string, DownstreamNode> downstreams;

        void from_parameters(MasterNode* node);
    };

    class MasterNodeRuntimeConfig {
    public:
        virtual ~MasterNodeRuntimeConfig(){}
        double frame_internal_ms = -1; // TODO: add more runtime configurations
        void from_parameters(MasterNode* node);
    };
}