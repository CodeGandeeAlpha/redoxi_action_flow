#pragma once

#include <json_struct/json_struct.h>
#include <typeinfo>
#include <redoxi_common_nodes/visibility_control.h>
#include <redoxi_common_nodes/base_nodes/OpenCloseNode.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <redoxi_common_nodes/image_ports/AsyncImageInputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionRequestOutputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionResponseOutputPort.hpp>


namespace redoxi_works::common_nodes::drivers
{

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
struct CallActionDriverInitConfig;

template <AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
struct CallActionDriverRuntimeConfig;

/*
 * CallActionDriverBase is a base class for driver nodes that call an action server to process requests.
 * It provides a framework for setting up the input port, callee port, and output port,
 * as well as defining the callbacks for processing input requests and callee results.
 * It can also be used as a standalone node, using callback functions to execute the processing.
 */
template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
class CallActionDriverBase : public OpenCloseNode
{
  public:
    using InitConfig_t = CallActionDriverInitConfig<InputPortType, CalleeRequestPortType, OutputPortType>;
    using RuntimeConfig_t = CallActionDriverRuntimeConfig<CalleeRequestPortType, OutputPortType>;

    using BaseNode_t = OpenCloseNode;
    using BaseInitConfig_t = BaseNode_t::InitConfig_t;
    using BaseRuntimeConfig_t = BaseNode_t::RuntimeConfig_t;
    using BaseNode_t::BaseNode_t;

  public:
    //! types of the input port of this node, which accepts incoming requests
    struct InputTypes {
        using InputPortSpec_t = typename InputPortType::MasterSpec_t;
        using InputPort_t = InputPortType;
        using Action_t = typename InputPortSpec_t::ActionType_t;
        using ActionDataTrait_t = typename InputPortSpec_t::ActionDataTrait_t;
        using ActionGoal_t = typename Action_t::Goal;
        using ActionResult_t = typename Action_t::Result;
        using SourceData_t = typename InputPortSpec_t::ReceiveSourceData_t;
    };

    //! Types for the callee, that is, the node with action server that processes the request
    struct CalleeTypes {
        using RequestOutputPort_t = CalleeRequestPortType;
        using RequestOutputPortSpec_t = typename RequestOutputPort_t::MasterSpec_t;
        using RequestOutputSourceData_t = typename RequestOutputPort_t::SourceData_t;
        using RequestOutputRequest_t = typename RequestOutputPort_t::DeliveryRequest_t;
        using RequestOutputAction_t = typename RequestOutputPortSpec_t::ActionType_t;
        using RequestOutputActionDataTrait_t = typename RequestOutputPortSpec_t::ActionDataTrait_t;
        using RequestOutputActionGoal_t = typename RequestOutputAction_t::Goal;
        using RequestOutputActionResult_t = typename RequestOutputAction_t::Result;
        using RequestOutputDeliveryPolicy_t = typename RequestOutputPort_t::DeliveryPolicy_t;
        using Downstream_t = typename RequestOutputPort_t::Downstream_t;
    };

    //! types of the output port of this node, which sends out actions to downstream nodes
    struct OutputTypes {
        using OutputPort_t = OutputPortType;
        using OutputPortSpec_t = typename OutputPort_t::MasterSpec_t;
        using OutputSourceData_t = typename OutputPort_t::SourceData_t;
        using OutputRequest_t = typename OutputPort_t::DeliveryRequest_t;
        using OutputAction_t = typename OutputPortSpec_t::ActionType_t;
        using OutputActionDataTrait_t = typename OutputPortSpec_t::ActionDataTrait_t;
        using OutputActionGoal_t = typename OutputAction_t::Goal;
        using OutputActionResult_t = typename OutputAction_t::Result;
        using OutputDeliveryPolicy_t = typename OutputPort_t::DeliveryPolicy_t;
    };

    using InputRequestHandler_t =
        port_handlers::PullProcessSendHandler<typename InputTypes::InputPortSpec_t,
                                              typename CalleeTypes::RequestOutputPortSpec_t>;

    //! Type alias for the data-processing callback of the input port handler
    using OnProcessInputRequestCallback_t = std::function<int(typename CalleeTypes::RequestOutputRequest_t *,
                                                              std::optional<typename CalleeTypes::RequestOutputDeliveryPolicy_t> *,
                                                              typename InputTypes::ActionResult_t *,
                                                              std::shared_ptr<const typename InputTypes::SourceData_t>,
                                                              typename InputRequestHandler_t::ResourceToken_t &)>;

    //! user defined callback for processing input request, called after the internal processing is done
    OnProcessInputRequestCallback_t on_process_input_request;

    //! Type alias for creating output request after receiving result from the callee
    using OnProcessCalleeResultCallback_t = std::function<int(typename OutputTypes::OutputRequest_t *,
                                                              typename OutputTypes::OutputDeliveryPolicy_t *,
                                                              std::shared_ptr<const typename CalleeTypes::RequestOutputActionResult_t>,
                                                              const typename CalleeTypes::RequestOutputRequest_t &,
                                                              const typename CalleeTypes::Downstream_t &)>;

    //! user defined callback for processing callee result, called after the internal processing is done
    OnProcessCalleeResultCallback_t on_process_callee_result;

  protected:
    int _open() override;
    int _close() override;
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

    DEFAULT_CONFIG_LOADER_IMPL(InitConfig_t, RuntimeConfig_t);

    /**
     * @brief After the request is received from the input port, process it
     * @details This is the data-processing callback of the input port handler
     * @param[out] out_callee_request The request to be sent to the callee,
     *        with the meta info (e.g., signal code, task metadata, etc) copied from source data
     * @param[out] out_callee_enqueue_policy The delivery policy for the callee request,
     *        already set to handle special signal code correctly
     * @param[out] out_upstream_result The result to be sent back upstream
     * @param[in] source_data The source data received from upstream
     * @param[in] resource_token Resource token for accessing shared resources
     * @return 0 on success, non-zero on failure
     */
    virtual int _on_process_input_request(typename CalleeTypes::RequestOutputRequest_t *out_callee_request,
                                          std::optional<typename CalleeTypes::RequestOutputDeliveryPolicy_t> *out_callee_enqueue_policy,
                                          typename InputTypes::ActionResult_t *out_upstream_result,
                                          std::shared_ptr<const typename InputTypes::SourceData_t> source_data,
                                          typename InputRequestHandler_t::ResourceToken_t &resource_token);

    /**
     * @brief After receiving result from callee, create output request for sending to output port
     * @details You can also modify the output request enqueue policy here, or just leave it unchanged
     * @param[out] out_downstream_request The output request to be sent downstream, with the meta info copied from callee request
     * @param[out] out_downstream_enqueue_policy The delivery policy for the output request, already set to handle special signal code correctly
     * @param[in] callee_result The result received from the callee
     * @param[in] callee_request The original request sent to the callee
     * @param[in] downstream The downstream information
     * @return 0 on success, non-zero on failure
     */
    virtual int _on_process_callee_result(typename OutputTypes::OutputRequest_t *out_downstream_request,
                                          typename OutputTypes::OutputDeliveryPolicy_t *out_downstream_enqueue_policy,
                                          std::shared_ptr<const typename CalleeTypes::RequestOutputActionResult_t> callee_result,
                                          const typename CalleeTypes::RequestOutputRequest_t &callee_request,
                                          const typename CalleeTypes::Downstream_t &downstream);

  protected:
    // for accepting incoming requests
    std::shared_ptr<typename InputTypes::InputPort_t> m_input_port;
    // for sending out actions
    std::shared_ptr<typename OutputTypes::OutputPort_t> m_output_port;
    // for sending requests to the callee
    std::shared_ptr<typename CalleeTypes::RequestOutputPort_t> m_callee_port;
    // for doing the work of pulling input and send it to callee
    std::shared_ptr<InputRequestHandler_t> m_input_request_handler;

  private:
    //! after the request is sent to the callee
    //! if you want to get result from goal handle, do it here
    //! you can async_get_result() and then choose to wait synchronously or asynchronously
    void _internal_request_sent_to_callee(typename CalleeTypes::RequestOutputPort_t::TargetData_t &target_data,
                                          typename CalleeTypes::RequestOutputPort_t::SendResult_t &send_result,
                                          const typename CalleeTypes::RequestOutputPort_t::DeliveryRequest_t &delivery_request,
                                          const typename CalleeTypes::RequestOutputPort_t::Downstream_t &downstream);

    //! after the request is received from the input port
    //! this is the data-processing callback of the input port handler, so policy will be set correctly to handle special signal code
    //! unless you override it in the callback
    int _internal_process_input_request(typename CalleeTypes::RequestOutputRequest_t *out_callee_request,
                                        std::optional<typename CalleeTypes::RequestOutputDeliveryPolicy_t> *out_callee_enqueue_policy,
                                        typename InputTypes::ActionResult_t *out_upstream_result,
                                        std::shared_ptr<const typename InputTypes::SourceData_t> source_data,
                                        typename InputRequestHandler_t::ResourceToken_t &resource_token)
    {
        // pass along task metadata and signal code
        auto task_metadata = InputTypes::ActionDataTrait_t::get_source_task_metadata(*source_data->get_goal());
        auto signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
        out_callee_request->set_source_task_metadata(task_metadata);
        out_callee_request->set_control_signal_code(signal_code);

        // TODO: request uuid?

        // call class member function
        auto ret_member = _on_process_input_request(out_callee_request,
                                                    out_callee_enqueue_policy,
                                                    out_upstream_result,
                                                    source_data,
                                                    resource_token);
        if (ret_member != 0) {
            RDX_INFO_DEV(this, __func__, false, "Error in internal processing of input request, ret={}", ret_member);
            return ret_member;
        }

        // then call user-defined callback
        int ret_user = 0;
        if (on_process_input_request) {
            ret_user = on_process_input_request(out_callee_request,
                                                out_callee_enqueue_policy,
                                                out_upstream_result,
                                                source_data,
                                                resource_token);
        }
        if (ret_user != 0) {
            RDX_INFO_DEV(this, __func__, false, "Error in user-defined processing of input request, ret={}", ret_user);
            return ret_user;
        }

        return 0;
    }

    //! after the result is received from the callee, create output request for sending to output port
    //! you can also modify the output request enqueue policy here, or just leave it unchanged
    int _internal_process_callee_result(typename OutputTypes::OutputRequest_t *out_downstream_request,
                                        typename OutputTypes::OutputDeliveryPolicy_t *out_downstream_enqueue_policy,
                                        std::shared_ptr<const typename CalleeTypes::RequestOutputActionResult_t> callee_result,
                                        const typename CalleeTypes::RequestOutputRequest_t &callee_request,
                                        const typename CalleeTypes::Downstream_t &downstream)
    {
        // pass along meta data from callee request
        out_downstream_request->copy_meta_info_from(callee_request);

        // call class member function
        auto ret_member = _on_process_callee_result(out_downstream_request,
                                                    out_downstream_enqueue_policy,
                                                    callee_result,
                                                    callee_request,
                                                    downstream);
        if (ret_member != 0) {
            RDX_INFO_DEV(this, __func__, false, "Error in internal processing of callee result, ret={}", ret_member);
            return ret_member;
        }

        // then call user-defined callback
        int ret_user = 0;
        if (on_process_callee_result) {
            ret_user = on_process_callee_result(out_downstream_request,
                                                out_downstream_enqueue_policy,
                                                callee_result,
                                                callee_request,
                                                downstream);
        }
        if (ret_user != 0) {
            RDX_INFO_DEV(this, __func__, false, "Error in user-defined processing of callee result, ret={}", ret_user);
            return ret_user;
        }

        return 0;
    }
};

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
struct CallActionDriverInitConfig : public OpenCloseNode::InitConfig_t {
    using DriverInputPortConfig_t = typename InputPortType::InitConfig_t;
    using DriverOutputPortConfig_t = typename OutputPortType::InitConfig_t;
    using CalleeRequestPortConfig_t = typename CalleeRequestPortType::InitConfig_t;

    //! config for the driver's input port
    std::shared_ptr<DriverInputPortConfig_t> input_port_config = std::make_shared<DriverInputPortConfig_t>();
    //! config for the driver's output port
    std::shared_ptr<DriverOutputPortConfig_t> output_port_config = std::make_shared<DriverOutputPortConfig_t>();
    //! config for the callee's request port
    std::shared_ptr<CalleeRequestPortConfig_t> callee_request_port_config = std::make_shared<CalleeRequestPortConfig_t>();

    JS_OBJECT_WITH_SUPER(JS_SUPER(OpenCloseNode::InitConfig_t),
                         JS_MEMBER(input_port_config),
                         JS_MEMBER(output_port_config),
                         JS_MEMBER(callee_request_port_config));
};

template <AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
struct CallActionDriverRuntimeConfig : public OpenCloseNode::RuntimeConfig_t {
    using CalleeRequestEnqueuePolicy_t = typename CalleeRequestPortType::DeliveryPolicy_t;
    using DriverOutputEnqueuePolicy_t = typename OutputPortType::DeliveryPolicy_t;

    //! policy when enqueueing to the callee's request port
    CalleeRequestEnqueuePolicy_t callee_request_enqueue_policy;

    //! policy when enqueueing to the driver's output port
    DriverOutputEnqueuePolicy_t driver_output_enqueue_policy;

    //! if true, the driver will block the incoming requests until the previous request is processed
    bool enable_blocking_mode = false;

    JS_OBJECT_WITH_SUPER(JS_SUPER(OpenCloseNode::RuntimeConfig_t),
                         JS_MEMBER(callee_request_enqueue_policy),
                         JS_MEMBER(driver_output_enqueue_policy),
                         JS_MEMBER(enable_blocking_mode));
};

// inline CallActionDriverBase::CallActionDriverBase(const std::string &name, const rclcpp::NodeOptions &options)
//     : OpenCloseNode(name, options)
// {
// }

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
int CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _open()
{
    RDX_INFO_DEV(this, __func__, false, "Opening driver node, class={}", typeid(*this).name());
    RDX_INFO_DEV(this, __func__, false, "Driver node opened, class={}", typeid(*this).name());
    return 0;
}

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
int CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _start()
{
    RDX_INFO_DEV(this, __func__, false, "Starting driver node, class={}", typeid(*this).name());
    if (m_input_port) {
        m_input_port->start();
    }
    if (m_callee_port) {
        m_callee_port->start();
    }
    if (m_output_port) {
        m_output_port->start();
    }
    RDX_INFO_DEV(this, __func__, false, "Driver node started, class={}", typeid(*this).name());
    return 0;
}

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
int CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _stop()
{
    RDX_INFO_DEV(this, __func__, false, "Stopping driver node, class={}", typeid(*this).name());
    if (m_input_port) {
        m_input_port->stop();
    }
    if (m_callee_port) {
        m_callee_port->stop();
    }
    if (m_output_port) {
        m_output_port->stop();
    }
    RDX_INFO_DEV(this, __func__, false, "Driver node stopped, class={}", typeid(*this).name());
    return 0;
}

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
int CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _close()
{
    RDX_INFO_DEV(this, __func__, false, "Closing driver node, class={}", typeid(*this).name());
    RDX_INFO_DEV(this, __func__, false, "Driver node closed, class={}", typeid(*this).name());
    return 0;
}

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
void CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _step()
{
    if (m_input_request_handler) {
        m_input_request_handler->process_and_send();
    }
}

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
int CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config)
{
    //! Update initial configuration
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting update of initial configuration");
    auto config = std::dynamic_pointer_cast<InitConfig_t>(init_config);
    if (!config) {
        RDX_RAISE_ERROR("Invalid init config, expected type: {}", typeid(InitConfig_t).name());
    }

    //! Create input port
    if (config->input_port_config && !config->input_port_config->get_action_name().empty()) {
        RDX_INFO_DEV(this, __func__, false, "Creating input port with action name: {}", config->input_port_config->get_action_name());
        m_input_port = std::make_shared<typename InputTypes::InputPort_t>(this);
        m_input_port->init(config->input_port_config);
    } else {
        RDX_INFO_DEV(this, __func__, false, "{}", "No input port config, skipping input port creation");
    }

    //! Create callee port
    if (config->callee_request_port_config) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Creating callee request output port");
        m_callee_port = std::make_shared<typename CalleeTypes::RequestOutputPort_t>(this);
        m_callee_port->init(config->callee_request_port_config);
    } else {
        RDX_INFO_DEV(this, __func__, false, "{}", "No callee request port config, skipping callee request port creation");
    }

    //! Create output port
    if (config->output_port_config) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Creating output port");
        m_output_port = std::make_shared<typename OutputTypes::OutputPort_t>(this);
        m_output_port->init(config->output_port_config);
    } else {
        RDX_INFO_DEV(this, __func__, false, "{}", "No output port config, skipping output port creation");
    }

    RDX_INFO_DEV(this, __func__, false, "{}", "Completed update of initial configuration");
    return 0;
}

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
int CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config)
{

    RDX_INFO_DEV(this, __func__, false, "{}", "Updating runtime configuration");
    //! Update runtime configuration
    auto config = std::dynamic_pointer_cast<RuntimeConfig_t>(runtime_config);
    if (!config) {
        RDX_RAISE_ERROR("Invalid runtime config, expected type: {}", typeid(RuntimeConfig_t).name());
    }

    if (m_callee_port) {
        m_callee_port->set_callback_on_deliver_to_downstream_finish(
            std::bind(&CallActionDriverBase::_internal_request_sent_to_callee, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4));
    }

    // create port handler
    if (m_input_port && m_callee_port) {
        using PortHandler_t = InputRequestHandler_t;
        auto handler_config = std::make_shared<typename PortHandler_t::InitConfig_t>();
        handler_config->block_input_reading = config->enable_blocking_mode;
        handler_config->block_resource_acquisition = config->enable_blocking_mode;
        m_input_request_handler = std::make_shared<PortHandler_t>();

        RDX_INFO_DEV(this, __func__, false, "{}", "Initializing port handler");
        m_input_request_handler->init(
            m_input_port.get(),
            m_callee_port.get(),
            nullptr,
            handler_config,
            config->callee_request_enqueue_policy);

        m_input_request_handler->on_process_input_data =
            std::bind(&CallActionDriverBase::_internal_process_input_request, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4,
                      std::placeholders::_5);
    }

    RDX_INFO_DEV(this, __func__, false, "{}", "Completed update of runtime configuration");
    return 0;
}

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
int CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _on_process_callee_result(typename OutputTypes::OutputRequest_t *out_downstream_request,
                              typename OutputTypes::OutputDeliveryPolicy_t *out_downstream_enqueue_policy,
                              std::shared_ptr<const typename CalleeTypes::RequestOutputActionResult_t> callee_result,
                              const typename CalleeTypes::RequestOutputRequest_t &callee_request,
                              const typename CalleeTypes::Downstream_t &downstream)
{
    (void)out_downstream_request;
    (void)out_downstream_enqueue_policy;
    (void)callee_result;
    (void)callee_request;
    (void)downstream;
    return 0;
}

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
void CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _internal_request_sent_to_callee(
        typename CalleeTypes::RequestOutputPort_t::TargetData_t &target_data,
        typename CalleeTypes::RequestOutputPort_t::SendResult_t &send_result,
        const typename CalleeTypes::RequestOutputPort_t::DeliveryRequest_t &delivery_request,
        const typename CalleeTypes::RequestOutputPort_t::Downstream_t &downstream)
{
    (void)delivery_request;
    auto msg_uuid = target_data.get_source_data_uuid();
    auto msg_uuid_str = boost::uuids::to_string(msg_uuid);

    if (send_result.goal_handle) {
        auto goal_handle = send_result.goal_handle;
        if (goal_handle) {
            auto result = downstream.get_action_client()->async_get_result(goal_handle).get();

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Got request result, sending to output port", msg_uuid_str);
                auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(get_runtime_config());
                auto policy = runtime_config->driver_output_enqueue_policy;
                auto signal_code = delivery_request.get_control_signal_code();

                // abnormal signal must be delivered reliably
                typename OutputTypes::OutputRequest_t response_request;
                if (signal_code != ControlSignalCode::Normal && signal_code != ControlSignalCode::Ping) {
                    policy.set_precondition(DeliveryPrecondition::NoPrecondition);
                    policy.set_drop_strategy(DropStrategy::NoDrop);
                    response_request.make_reliable();
                }

                if (m_output_port) {
                    auto ret = _internal_process_callee_result(&response_request, &policy, result.result, delivery_request, downstream);
                    if (ret == 0) {
                        auto sent = m_output_port->push_request(response_request, policy);
                        if (sent) {
                            RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Sent output request to output port", msg_uuid_str);
                        } else {
                            RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Failed to send output request to output port", msg_uuid_str);
                        }
                    } else {
                        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Failed to process callee result, skip sending", msg_uuid_str);
                    }
                }
            } else {
                RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Failed to retrieve request result", msg_uuid_str);
            }
        } else {
            RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Failed to get goal handle for request", msg_uuid_str);
        }
    }
}

template <AsyncActionInputPortConcept InputPortType,
          AsyncActionOutputPortConcept CalleeRequestPortType,
          AsyncActionOutputPortConcept OutputPortType>
int CallActionDriverBase<InputPortType, CalleeRequestPortType, OutputPortType>::
    _on_process_input_request(typename CalleeTypes::RequestOutputRequest_t *out_callee_request,
                              std::optional<typename CalleeTypes::RequestOutputDeliveryPolicy_t> *out_callee_enqueue_policy,
                              typename InputTypes::ActionResult_t *out_upstream_result,
                              std::shared_ptr<const typename InputTypes::SourceData_t> source_data,
                              typename InputRequestHandler_t::ResourceToken_t &resource_token)
{
    (void)out_callee_request;
    (void)out_callee_enqueue_policy;
    (void)out_upstream_result;
    (void)source_data;
    (void)resource_token;
    return 0;
}

} // namespace redoxi_works::common_nodes::drivers
