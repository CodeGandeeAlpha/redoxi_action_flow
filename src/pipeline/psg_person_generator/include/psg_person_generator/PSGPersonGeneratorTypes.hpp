#pragma once

#include <psg_document_sink/AsyncDocumentInputPort.hpp>
#include <psg_document_sink/DocumentInputSpec.hpp>
#include <psg_master_node/AsyncDocumentOutputPort.hpp>
#include <psg_master_node/DocumentOutputSpec.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works
{
class PSGPersonGenerator;

namespace psg_person_generator
{
using InputPortType = AsyncDocumentInputPort;
using OutputPortType = AsyncDocumentOutputPort;
using OutputPortSpec = OutputPortType::MasterSpec_t;

//! The delivery policy for making frame delivery request
using RequestPolicy = OutputPortSpec::DeliveryPolicy_t;

//! The init config for PSGPersonGenerator
struct InitConfig {
    virtual ~InitConfig() = default;
    InitConfig()
    {
        // by default, only starts sending any data when some downstream is ready
        // skip if no downstream is ready
        output_port_config.set_fallback_delivery_precondition(DeliveryPrecondition::AnyDownstreamReady);
    }

    //! The time unit for the step interval, see redoxi_common_cpp::get_default_time_unit_name for more details
    //! @note: this is just for json serialization, intended as a comment, do not modify it
    std::optional<std::string> _time_unit = get_default_time_unit_name();

    std::shared_ptr<InputPortType::InitConfig_t>
        input_port_config = std::make_shared<InputPortType::InitConfig_t>();

    //! The downstream nodes, indexed by node name
    OutputPortSpec::InitConfig_t output_port_config;

    //! Use blocking mode for the reading input port
    bool enable_blocking_mode = false;

    //! create the debug publish topic for this node?
    bool create_debug_pub = true;
    int debug_pub_queue_size = 10;
    std::string debug_pub_task_enqueue_name = "debug_port/task_enqueue";
    std::string debug_pub_task_drop_name = "debug_port/task_drop";

    //! Load parameters from node, this will override empty existing parameters
    virtual void from_parameters(PSGPersonGenerator *);

    // json serialize
    JS_OBJECT(JS_MEMBER(_time_unit),
              JS_MEMBER(input_port_config),
              JS_MEMBER(output_port_config),
              JS_MEMBER(create_debug_pub),
              JS_MEMBER(enable_blocking_mode),
              JS_MEMBER(debug_pub_queue_size),
              JS_MEMBER(debug_pub_task_enqueue_name),
              JS_MEMBER(debug_pub_task_drop_name));
};

//! The runtime config for PSGPersonGenerator
class RuntimeConfig
{
  public:
    inline static const DefaultTimeUnit_t DEFAULT_STEP_INTERVAL{std::chrono::milliseconds(5)};
    inline static const DefaultTimeUnit_t DEFAULT_REQUEST_RETRY_INTERVAL{std::chrono::milliseconds(5)};
    inline static const DefaultTimeUnit_t DEFAULT_REQUEST_RETRY_RESPONSE_TIME{std::chrono::milliseconds(5)};
    inline static const int DEFAULT_REQUEST_RETRY_NUMBER{5};

    virtual ~RuntimeConfig() = default;
    RuntimeConfig()
    {
        auto &p = frame_enqueue_policy;
        p.set_drop_strategy(DropStrategy::DropAsNeeded);
        p.set_precondition(DeliveryPrecondition::AnyDownstreamReady);
        p.get_retry_policy().set_number_of_retry(DEFAULT_REQUEST_RETRY_NUMBER);
        p.get_retry_policy().set_wait_time_between_retry(DEFAULT_REQUEST_RETRY_INTERVAL);
        p.get_retry_policy().set_wait_time_retry_response(DEFAULT_REQUEST_RETRY_RESPONSE_TIME);
    }

    //! The time unit for the step interval, see redoxi_common_cpp::get_default_time_unit_name for more details
    //! @note: this is just for json serialization, do not modify it
    std::optional<std::string> _time_unit = get_default_time_unit_name();

    //! The step interval in ms
    OutputPortSpec::TimeUnit_t step_interval{DEFAULT_STEP_INTERVAL};

    //! The frame interval in ms, 0 means as fast as possible
    OutputPortSpec::TimeUnit_t frame_interval{0};

    //! publish in debug topic?
    bool publish_to_debug_topic = false;

    //! delivery policy for frame delivery request, after the frame is enqueued
    //! when this is set, it will override the individual downstream delivery policies in the output port
    std::optional<OutputPortSpec::DeliveryPolicy_t> frame_request_policy;

    //! delivery policy for frame enqueue request
    RequestPolicy frame_enqueue_policy;

    //! Load parameters from node, this will override empty existing parameters
    virtual void from_parameters(PSGPersonGenerator *);

    // json serialize
    JS_OBJECT(JS_MEMBER(_time_unit),
              JS_MEMBER(step_interval),
              JS_MEMBER(frame_interval),
              JS_MEMBER(publish_to_debug_topic),
              JS_MEMBER(frame_request_policy),
              JS_MEMBER(frame_enqueue_policy));
};

} // namespace psg_person_generator

} // namespace redoxi_works
