#pragma once

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <redoxi_common_nodes/image_ports/AsyncImageInputPort.hpp>
#include <psg_master_node/AsyncDocumentOutputPort.hpp>
#include <psg_master_node/DocumentOutputSpec.hpp>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works
{
class PSGMasterNode;

namespace psg_master_node
{
using InputPortType = image_ports::AsyncImageInputPort;
using OutputPortType = AsyncDocumentOutputPort;
using OutputPortSpec = OutputPortType::MasterSpec_t;

//! The delivery policy for making frame delivery request
using RequestPolicy = OutputPortSpec::DeliveryPolicy_t;

//! The init config for PSGMasterNode
struct InitConfig : public common_nodes::StartStopNode::InitConfig_t {
    virtual ~InitConfig() = default;
    InitConfig()
    {
        // by default, only starts sending any data when some downstream is ready
        // skip if no downstream is ready
        output_port_config->set_fallback_delivery_precondition(DeliveryPrecondition::AnyDownstreamReady);
    }

    std::shared_ptr<InputPortType::InitConfig_t>
        input_port_config = std::make_shared<InputPortType::InitConfig_t>();

    //! The downstream nodes, indexed by node name
    std::shared_ptr<OutputPortSpec::InitConfig_t> output_port_config = std::make_shared<OutputPortSpec::InitConfig_t>();

    //! create the debug publish topic for this node?
    bool create_debug_pub = true;
    int debug_pub_queue_size = 10;
    std::string debug_pub_task_enqueue_name = "debug_port/task_enqueue";
    std::string debug_pub_task_drop_name = "debug_port/task_drop";

    //! parse from node, the node must be exactly PSGMasterNode, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, PSGMasterNode>
    void from_node(const Node_t *node)
    {
        InitConfig::parse_from_node_parameters(this, node);
    }

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
                         JS_MEMBER(input_port_config),
                         JS_MEMBER(output_port_config),
                         JS_MEMBER(create_debug_pub),
                         JS_MEMBER(debug_pub_queue_size),
                         JS_MEMBER(debug_pub_task_enqueue_name),
                         JS_MEMBER(debug_pub_task_drop_name));
};

//! The runtime config for PSGMasterNode
struct RuntimeConfig : public common_nodes::StartStopNode::RuntimeConfig_t {
  public:
    // IMPORTANT: default output is rgb8, if you use bgr8, you need to specify it in the config
    inline static const TimeUnit_t DefaultRequestRetryInterval{std::chrono::milliseconds(5)};
    inline static const TimeUnit_t DefaultRequestRetryResponseTime{std::chrono::milliseconds(5)};
    inline static const int DefaultRequestRetryNumber{5};

    virtual ~RuntimeConfig() = default;
    RuntimeConfig()
    {
        auto &p = frame_enqueue_policy;
        p.set_drop_strategy(DropStrategy::DropAsNeeded);
        p.set_precondition(DeliveryPrecondition::AnyDownstreamReady);
        p.get_retry_policy().set_number_of_retry(DefaultRequestRetryNumber);
        p.get_retry_policy().set_wait_time_between_retry(DefaultRequestRetryInterval);
        p.get_retry_policy().set_wait_time_retry_response(DefaultRequestRetryResponseTime);
    }

    //! The frame interval in ms, 0 means as fast as possible
    TimeUnit_t frame_interval{0};

    //! publish in debug topic?
    bool publish_to_debug_topic = false;

    //! delivery policy for frame delivery request, after the frame is enqueued
    //! when this is set, it will override the individual downstream delivery policies in the output port
    std::optional<OutputPortSpec::DeliveryPolicy_t> frame_request_policy;

    //! delivery policy for frame enqueue request
    RequestPolicy frame_enqueue_policy;

    //! enable blocking mode?
    bool enable_blocking_mode = false;

    //! enable debug topic?
    bool enable_debug_topic = false;

    //! parse from node, the node must be exactly PSGMasterNode, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, PSGMasterNode>
    void from_node(const Node_t *node)
    {
        RuntimeConfig::parse_from_node_parameters(this, node);
    }

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::RuntimeConfig_t),
                         JS_MEMBER(frame_interval),
                         JS_MEMBER(publish_to_debug_topic),
                         JS_MEMBER(frame_request_policy),
                         JS_MEMBER(frame_enqueue_policy),
                         JS_MEMBER(enable_blocking_mode),
                         JS_MEMBER(enable_debug_topic));
};

} // namespace psg_master_node

} // namespace redoxi_works
