#pragma once
#include <string>
#include <map>
#include <psg_common/psg_common.hpp>

namespace FlowRos2Pipeline {
    class DetectorIn;

    class DetectorInDownstreamPipelineNode{
    public:
        virtual ~DetectorInDownstreamPipelineNode(){}
        std::string accept_document_action;
    };

    class DetectorInDownstreamModelNode{
    public:
        virtual ~DetectorInDownstreamModelNode(){}
        std::string accept_frame_action;
    };

    class DetectorInInitConfig{
    public:
        virtual ~DetectorInInitConfig(){}

        //downstream nodes, mapping node name to node configurations
        using DownstreamPipelineNode = DetectorInDownstreamPipelineNode;
        std::map<std::string, DownstreamPipelineNode> pipeline_downstreams;

        using DownstreamModelNode = DetectorInDownstreamModelNode;
        std::map<std::string, DownstreamModelNode> model_downstreams;

        std::string process_document_action;

        void from_parameters(DetectorIn* node);
    };

    class DetectorInRuntimeConfig{
    public:
        virtual ~DetectorInRuntimeConfig(){}

        // step frequency
        double step_interval_ms = DefaultNodeStepIntervalMs;

        double timeout_ms_send_to_downstream = DefaultTimeoutMs;

        void from_parameters(DetectorIn* node);
    };
}