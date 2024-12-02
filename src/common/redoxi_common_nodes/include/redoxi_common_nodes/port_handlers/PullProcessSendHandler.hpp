#pragma once

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputPort.hpp>
// #include <redoxi_common_nodes/image_ports/AsyncImageInputPort.hpp>
// #include <redoxi_common_nodes/image_ports/AsyncImageOutputPort.hpp>
#include <tbb/concurrent_queue.h>

namespace redoxi_works::port_handlers
{
// using InputPortSpecType = redoxi_works::image_ports::types::ImageActionInputPortSpec;
// using OutputPortSpecType = redoxi_works::image_ports::types::ImageActionOutputPortSpec;

// struct ResourceTokenType {
// };

template <input_port_types::AsyncActionInputPortSpecConcept InputPortSpecType,
          output_port_types::AsyncActionOutputPortSpecConcept OutputPortSpecType,
          ResourceTokenConcept ResourceTokenType = DummyResourceToken>
class PullProcessSendHandler
{
  public:
    using InitConfig_t = PullProcessSendHandlerConfig;
    using ResourceToken_t = ResourceTokenType;
    using ResourceTokenQueue_t = tbb::concurrent_bounded_queue<ResourceToken_t>;

    using InputPortSpec_t = InputPortSpecType;
    using InputPort_t = AsyncActionInputPort<InputPortSpec_t>;
    using InputAction_t = typename InputPortSpec_t::ActionType_t;
    using InputActionResult_t = typename InputAction_t::Result;
    using InputSourceData_t = typename InputPort_t::SourceData_t;
    using InputGoalHandle_t = typename InputSourceData_t::GoalHandle_t;
    using InputActionDataTrait_t = typename InputPortSpec_t::ActionDataTrait_t;

    using OutputPortSpec_t = OutputPortSpecType;
    using OutputPort_t = AsyncActionOutputPort<OutputPortSpec_t>;
    using OutputSourceData_t = typename OutputPort_t::SourceData_t;
    using OutputRequest_t = typename OutputPort_t::DeliveryRequest_t;
    using OutputDeliveryPolicy_t = typename OutputPort_t::DeliveryPolicy_t;

    using InputDataProcessCallback_t = std::function<int(OutputRequest_t *output_request,
                                                         InputActionResult_t *action_result,
                                                         std::shared_ptr<InputSourceData_t> source_data,
                                                         ResourceToken_t &resource_token)>;

    enum class ProcessResult {
        Success = 0,
        NoData = 1,
        NoResourceToken = 2,
        FailedToSend = 3,
        Error = -1,
    };

  public:
    PullProcessSendHandler() = default;
    virtual ~PullProcessSendHandler() = default;

    virtual void init(
        InputPort_t *input_port,
        OutputPort_t *output_port,
        ResourceTokenQueue_t *resource_token_queue,
        std::shared_ptr<PullProcessSendHandlerConfig> config,
        std::optional<OutputDeliveryPolicy_t> output_delivery_policy = std::nullopt,
        rclcpp::Node *node = nullptr)
    {
        m_resource_token_queue = resource_token_queue;
        m_input_port = input_port;
        m_output_port = output_port;
        m_output_delivery_policy = output_delivery_policy;
        m_config = config;
        m_node = node;
    }

    // get data from input port, process it, and send to output port
    // return 0 if success, -1 if failed
    virtual ProcessResult process_and_send()
    {
        if (!m_input_port) {
            return ProcessResult::NoData;
        }

        // RDX_INFO_DEV(nullptr, __func__, true, "{}", "Trying to get input data");
        // get data from input port
        std::shared_ptr<InputSourceData_t> input_data;
        if (m_config->block_input_reading) {
            input_data = m_input_port->pop_source_data();
        } else {
            input_data = m_input_port->try_pop_source_data();
        }
        if (!input_data) {
            // RDX_INFO_DEV(nullptr, __func__, true, "{}", "No input data");
            return ProcessResult::NoData;
        }

        // get goal handle
        auto goal_handle = input_data->get_goal_handle_future().get();
        if (!goal_handle) {
            RDX_INFO_DEV(nullptr, __func__, true, "{}", "Goal handle not found");
            // goal handle is not found, which means the goal is not accepted, should not happen
            return ProcessResult::Error;
        }
        auto msg_uuid = InputActionDataTrait_t::get_uuid(*input_data->get_goal());
        auto msg_uuid_str = UUIDTrait::to_string(msg_uuid);
        RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Got goal handle", msg_uuid_str);

        // get resource token
        ResourceToken_t resource_token;
        bool got_resource_token = true;
        if (m_resource_token_queue) {
            if (m_config->block_resource_acquisition) {
                m_resource_token_queue->pop(resource_token);
                got_resource_token = true;
            } else {
                got_resource_token = m_resource_token_queue->try_pop(resource_token);
            }
            if (!got_resource_token) {
                RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] No resource token", msg_uuid_str);
                // notify the user that no resource token is available
                if (on_resource_token_not_available) {
                    on_resource_token_not_available(input_data);
                }
                return ProcessResult::NoResourceToken;
            }
            RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Got resource token", msg_uuid_str);
        }

        auto release_token = [this, msg_uuid_str](ResourceToken_t &token) {
            bool do_release = true;
            if (on_release_resource_token) {
                do_release = on_release_resource_token(token) == 0;
            }
            if (do_release && m_resource_token_queue) {
                RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Releasing resource token", msg_uuid_str);
                m_resource_token_queue->push(token);
            } else {
                RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] User handled resource token, not releasing it", msg_uuid_str);
            }
        };

        // work on the input data
        OutputRequest_t output_request;
        auto action_result = std::make_shared<InputActionResult_t>();
        if (on_process_input_data) {
            RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Processing input data", msg_uuid_str);
            int process_result = on_process_input_data(&output_request,
                                                       action_result.get(),
                                                       input_data,
                                                       resource_token);
            if (process_result != 0) {
                RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Processing input data failed, releasing resource token", msg_uuid_str);
                release_token(resource_token);
                try {
                    // FIXME: this might throw an exception if the goal on the client side is terminated
                    // saying: asked to send result for goal that does not exist
                    goal_handle->abort(action_result);
                } catch (const std::exception &e) {
                    RDX_LOG_ERROR(nullptr, __func__, true, "[msg_uuid={}] Failed to mark goal as aborted: {}", msg_uuid_str, e.what());
                }
                return ProcessResult::Error;
            }
        }

        // notify the user that input data is processed
        if (on_input_data_processed) {
            on_input_data_processed(output_request, resource_token);
        }

        // send data to output port
        if (m_output_port) {
            RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Sending data to output port", msg_uuid_str);
            bool sent = false;
            if (m_output_delivery_policy.has_value()) {
                sent = m_output_port->push_request(output_request, *m_output_delivery_policy);
            } else {
                sent = m_output_port->try_push_request(output_request);
            }
            if (!sent) {
                RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Failed to send data to output port, releasing resource token", msg_uuid_str);
                release_token(resource_token);
                try {
                    // FIXME: this might throw an exception if the goal on the client side is terminated
                    // saying: asked to send result for goal that does not exist
                    goal_handle->succeed(action_result);
                } catch (const std::exception &e) {
                    RDX_LOG_ERROR(nullptr, __func__, true, "[msg_uuid={}] Failed to mark goal as success: {}", msg_uuid_str, e.what());
                    return ProcessResult::Error;
                }
                return ProcessResult::FailedToSend;
            } else {
                RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Sent data to output port", msg_uuid_str);
            }
        }

        // ok, return resource token
        RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Returning resource token", msg_uuid_str);
        release_token(resource_token);

        // done
        RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Done, marking goal as success", msg_uuid_str);
        try {
            goal_handle->succeed(action_result);
        } catch (const std::exception &e) {
            RDX_LOG_ERROR(nullptr, __func__, true, "[msg_uuid={}] Failed to mark goal as success: {}", msg_uuid_str, e.what());
            return ProcessResult::Error;
        }
        return ProcessResult::Success;
    }

  public:
    //! Callback when processing input data
    InputDataProcessCallback_t on_process_input_data;

    //! Callback when input data is processed
    std::function<void(OutputRequest_t &request,
                       ResourceToken_t &resource_token)>
        on_input_data_processed;

    //! Callback when no resource token is available
    std::function<void(std::shared_ptr<InputSourceData_t> source_data)>
        on_resource_token_not_available;

    //! Callback before releasing resource token
    //! Return 0 if success, resource token will be released,
    //! return -1 if failed, resource token will not be released, you handle it yourself
    std::function<int(ResourceToken_t &resource_token)> on_release_resource_token;

  private:
    rclcpp::Node *m_node = nullptr;
    ResourceTokenQueue_t *m_resource_token_queue = nullptr;
    InputPort_t *m_input_port = nullptr;
    OutputPort_t *m_output_port = nullptr;
    std::optional<OutputDeliveryPolicy_t> m_output_delivery_policy;
    std::shared_ptr<PullProcessSendHandlerConfig> m_config = nullptr;
};


} // namespace redoxi_works::port_handlers
