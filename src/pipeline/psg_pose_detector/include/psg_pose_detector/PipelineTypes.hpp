#pragma once

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <psg_document_sink/AsyncDocumentInputPort.hpp>
#include <psg_master_node/AsyncDocumentOutputPort.hpp>
#include <psg_pose_detector/AsyncGetKeypointsOutputPort.hpp>
#include <psg_document_sink/DocumentInputSpec.hpp>
#include <psg_master_node/DocumentOutputSpec.hpp>
#include <psg_pose_detector/GetKeypointsOutputSpec.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works
{
class PSGPoseDetectorNode;

namespace psg_pose_detector
{
using InputPortType = AsyncDocumentInputPort;
using OutputPortPipelineType = AsyncDocumentOutputPort;
using OutputPortPipelineSpec = OutputPortPipelineType::MasterSpec_t;
using OutputPortModelType = AsyncGetKeypointsOutputPort;
using OutputPortModelSpec = OutputPortModelType::MasterSpec_t;

//! The delivery policy for making frame delivery request
using RequestPolicyPipeline = OutputPortPipelineSpec::DeliveryPolicy_t;
using RequestPolicyModel = OutputPortModelSpec::DeliveryPolicy_t;

//! The init config for PSGPoseDetectorNode
struct InitConfig : public common_nodes::StartStopNode::InitConfig_t {
    virtual ~InitConfig() = default;
    InitConfig()
    {
        // by default, only starts sending any data when some downstream is ready
        // skip if no downstream is ready
        output_port_pipeline_config->set_fallback_delivery_precondition(DeliveryPrecondition::AnyDownstreamReady);
    }

    std::shared_ptr<InputPortType::InitConfig_t>
        input_port_config = std::make_shared<InputPortType::InitConfig_t>();

    //! The downstream nodes, indexed by node name
    std::shared_ptr<OutputPortPipelineSpec::InitConfig_t> output_port_pipeline_config =
        std::make_shared<OutputPortPipelineSpec::InitConfig_t>();
    std::shared_ptr<OutputPortModelSpec::InitConfig_t> output_port_model_config =
        std::make_shared<OutputPortModelSpec::InitConfig_t>();


    //! create the debug publish topic for this node?
    bool create_debug_pub = true;
    int debug_pub_queue_size = 10;
    std::string debug_pub_pipeline_enqueue_name = "debug_port/pipeline_enqueue";
    std::string debug_pub_pipeline_drop_name = "debug_port/pipeline_drop";
    std::string debug_pub_model_enqueue_name = "debug_port/model_enqueue";
    std::string debug_pub_model_drop_name = "debug_port/model_drop";

    //! parse from node, the node must be exactly PSGPoseDetectorNode, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, PSGPoseDetectorNode>
    void from_node(const Node_t *node)
    {
        InitConfig::parse_from_node_parameters(this, node);
    }

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
                         JS_MEMBER(input_port_config),
                         JS_MEMBER(output_port_pipeline_config),
                         JS_MEMBER(output_port_model_config),
                         JS_MEMBER(create_debug_pub),
                         JS_MEMBER(debug_pub_queue_size),
                         JS_MEMBER(debug_pub_pipeline_enqueue_name),
                         JS_MEMBER(debug_pub_pipeline_drop_name),
                         JS_MEMBER(debug_pub_model_enqueue_name),
                         JS_MEMBER(debug_pub_model_drop_name));
};

//! The runtime config for PSGPoseDetectorNode
struct RuntimeConfig : public common_nodes::StartStopNode::RuntimeConfig_t {
  public:
    inline static const TimeUnit_t DefaultRequestRetryInterval{std::chrono::milliseconds(5)};
    inline static const TimeUnit_t DefaultRequestRetryResponseTime{std::chrono::milliseconds(5)};
    inline static const int DefaultRequestRetryNumber{5};

    virtual ~RuntimeConfig() = default;
    RuntimeConfig()
    {
        auto &p = pipeline_enqueue_policy;
        p.set_drop_strategy(DropStrategy::DropAsNeeded);
        p.set_precondition(DeliveryPrecondition::AnyDownstreamReady);
        p.get_retry_policy().set_number_of_retry(DefaultRequestRetryNumber);
        p.get_retry_policy().set_wait_time_between_retry(DefaultRequestRetryInterval);
        p.get_retry_policy().set_wait_time_retry_response(DefaultRequestRetryResponseTime);

        auto &m = model_enqueue_policy;
        m.set_drop_strategy(DropStrategy::DropAsNeeded);
        m.set_precondition(DeliveryPrecondition::AnyDownstreamReady);
        m.get_retry_policy().set_number_of_retry(DefaultRequestRetryNumber);
        m.get_retry_policy().set_wait_time_between_retry(DefaultRequestRetryInterval);
        m.get_retry_policy().set_wait_time_retry_response(DefaultRequestRetryResponseTime);
    }

    //! The document interval in ms, 0 means as fast as possible
    OutputPortPipelineSpec::TimeUnit_t document_interval{0};

    //! publish in debug topic?
    bool publish_to_debug_topic = false;

    //! delivery policy for document delivery request, after the document is enqueued
    //! when this is set, it will override the individual downstream delivery policies in the output port
    std::optional<OutputPortPipelineSpec::DeliveryPolicy_t> pipeline_request_policy;

    //! delivery policy for model delivery request, after the model is enqueued
    //! when this is set, it will override the individual downstream delivery policies in the output port
    std::optional<OutputPortModelSpec::DeliveryPolicy_t> model_request_policy;


    //! Use blocking mode for the reading input port
    bool enable_blocking_mode = false;
    //! delivery policy for document enqueue request
    RequestPolicyPipeline pipeline_enqueue_policy;

    //! delivery policy for model enqueue request
    RequestPolicyModel model_enqueue_policy;

    //! parse from node, the node must be exactly PSGPoseDetectorNode, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, PSGPoseDetectorNode>
    void from_node(const Node_t *node)
    {
        RuntimeConfig::parse_from_node_parameters(this, node);
    }

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::RuntimeConfig_t),
                         JS_MEMBER(document_interval),
                         JS_MEMBER(publish_to_debug_topic),
                         JS_MEMBER(pipeline_request_policy),
                         JS_MEMBER(model_request_policy),
                         JS_MEMBER(enable_blocking_mode),
                         JS_MEMBER(pipeline_enqueue_policy),
                         JS_MEMBER(model_enqueue_policy));
};

} // namespace psg_pose_detector

} // namespace redoxi_works
