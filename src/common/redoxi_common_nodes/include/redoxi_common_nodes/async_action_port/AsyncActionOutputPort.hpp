#pragma once

#include <string>
#include <atomic>
#include <rclcpp_action/rclcpp_action.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <redoxi_common_cpp/ros_utils/SyncActionSender.hpp>

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <redoxi_common_nodes/async_action_port/ImageOutputPortSpec.hpp>


namespace redoxi_works
{

using TSpec = async_action_image_output_port::ImageOutputPortSpec;

//! Sends action requests to downstream nodes, asynchronously
//! Thread safe, can be used in multi thread executor
// template <output_port_types::AsyncActionOutputPortSpecConcept TSpec>
class AsyncActionOutputPort : public IStartStopProtocol
{
  private:
    inline static constexpr auto PRINT_THREAD_ID = false;

  public:
    AsyncActionOutputPort() = default;
    virtual ~AsyncActionOutputPort() noexcept
    {
        // must finish all tasks
        if (m_delivery_graph != nullptr) {
            m_delivery_graph->wait_for_all();
        }
        m_task_group.wait();
    }

    using MasterSpec_t = TSpec; // master specification of this port
    using TimeUnit_t = typename TSpec::TimeUnit_t;
    using InitConfig_t = typename TSpec::InitConfig_t;
    using SourceData_t = typename TSpec::DeliverySourceData_t;
    using TargetData_t = typename TSpec::DeliveryTargetData_t;
    using DeliveryRequest_t = typename TSpec::DeliveryRequest_t;
    using DeliveryTask_t = typename TSpec::DeliveryTask_t;
    using Downstream_t = typename TSpec::Downstream_t;
    using DownstreamSpec_t = typename TSpec::DownstreamSpec_t;
    using ActionType_t = typename TSpec::ActionType_t;
    using ActionDataTrait_t = typename TSpec::ActionDataTrait_t;
    using DeliveryPolicy_t = typename DeliveryRequest_t::DeliveryPolicy_t;
    using RetryPolicy_t = typename DeliveryPolicy_t::RetryPolicyType_t;

    // synchronous action sender
    using SyncActionSender_t = SyncActionSender<ActionType_t>;

    // returned by sync action sender
    using SendResult_t = SyncActionSender_t::SendResult_t;

    //! Result of a delivery task
    struct DeliveryResult_t {
        //! Constructor from int
        explicit DeliveryResult_t(int result)
            : result_code(static_cast<DeliveryResultCode>(result))
        {
        }

        //! Constructor from DeliveryResultCode
        explicit DeliveryResult_t(DeliveryResultCode code)
            : result_code(code)
        {
        }

        //! Default constructor
        DeliveryResult_t() = default;

        DeliveryResultCode result_code{DeliveryResultCode::NotTried};
    };

  protected:
    using DeliveryTaskNode_t = async_processor::SingleBufferExecNode<DeliveryTask_t>;

  public:
    //! Try to push a request to the port
    //! @return true if success, otherwise false
    virtual bool try_push_request(const DeliveryRequest_t &request)
    {
        // must started first
        RDX_ASSERT_CHECK_TRUE(m_status == NodeStatusCode::STARTED, "[{}] must started first", __func__);

        DeliveryTask_t task;
        _create_frame_delivery_task(request, task);
        return m_delivery_task_node->put_data(task);
    }

    // wait for all requests to be processed
    void wait_for_all_requests()
    {
        if (m_delivery_graph != nullptr) {
            m_delivery_graph->wait_for_all();
        }
        m_task_group.wait();
    }

  public:
    /**
     * @brief Initialize the port
     * @note state transition: BEFORE_INIT -> STOPPED
     * @return 0 if success, otherwise return error code
     */
    virtual int init(const InitConfig_t &init_config)
    {
        // state must be BEFORE_INIT
        RDX_ASSERT_CHECK_TRUE(m_status == NodeStatusCode::BEFORE_INIT, "[{}] state must be BEFORE_INIT", __func__);

        // assign init config
        m_init_config = init_config;

        // do internal initialization
        {
            auto ret = _create_task_delivery_graph();
            if (ret != 0) {
                RDX_RAISE_ERROR("[{}] failed to create task delivery graph", __func__);
            }
        }

        // connect to downstreams
        {
            auto ret = _connect_to_downstreams();
            if (ret != 0) {
                RDX_RAISE_ERROR("[{}] failed to connect to downstreams", __func__);
            }
        }

        // state transition: BEFORE_INIT -> STOPPED
        _set_status_code(NodeStatusCode::STOPPED);

        return 0;
    }

    /**
     * @brief Start the port, begin sending action data to downstream nodes
     * @note you cannot modify the downstream specs after starting
     * @note state transition: STOPPED -> STARTED
     * @return 0 if success, otherwise return error code
     */
    virtual int start() override
    {
        // already started? skip
        if (m_status == NodeStatusCode::STARTED) {
            return 0;
        }

        // custom start logic
        {
            auto ret = _start();
            if (ret != 0) {
                RDX_RAISE_ERROR("[{}] failed to start the port", __func__);
            }
        }

        // state transition: STOPPED -> STARTED
        _set_status_code(NodeStatusCode::STARTED);

        return 0;
    }

    /**
     * @brief Stop the port from action, you can then modify the downstream specs
     * @note state transition: STARTED -> STOPPED
     * @return 0 if success, otherwise return error code
     */
    int stop() override
    {
        // already stopped? skip
        if (m_status == NodeStatusCode::STOPPED) {
            return 0;
        }
        // wait for all downstreams to finish
        if (m_delivery_graph != nullptr) {
            m_delivery_graph->wait_for_all();
        }

        // have any task running?
        m_task_group.wait();

        // custom stop logic
        {
            auto ret = _stop();
            if (ret != 0) {
                RDX_RAISE_ERROR("[{}] failed to stop the port", __func__);
            }
        }

        // state transition: STARTED -> STOPPED
        _set_status_code(NodeStatusCode::STOPPED);

        return 0;
    }

    // publish to debug topics?
    void set_publish_to_debug_topic(bool enable)
    {
        m_publish_to_debug_topic = enable;
    }

    bool get_publish_to_debug_topic() const
    {
        return m_publish_to_debug_topic;
    }

    // get the init config
    const InitConfig_t &get_init_config() const
    {
        return m_init_config;
    }
    InitConfig_t &get_init_config()
    {
        return m_init_config;
    }

    //! Get the status code of the port
    //! @return The status code
    int get_status() const
    {
        return m_status;
    }

  protected:
    //! for subclass to implement, executed during start(), before state transition
    //! @return 0 if success, otherwise return error code, which will cause start() to fail
    virtual int _start()
    {
        return 0;
    }

    //! for subclass to implement, executed during stop(), after state transition
    //! @return 0 if success, otherwise return error code, which will cause stop() to fail
    virtual int _stop()
    {
        return 0;
    }

  protected:
    //! Create the task delivery graph
    //! @return 0 if success, otherwise return error code
    virtual int _create_task_delivery_graph()
    {
        constexpr auto PRINT_THREAD_ID = false;

        // create graph and node
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Creating graph ...");
        m_delivery_graph = std::make_shared<tbb::flow::graph>();
        auto &g = *m_delivery_graph;

        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Creating frame delivery node ...");
        m_delivery_task_node = std::make_shared<
            async_processor::SingleBufferExecNode<DeliveryTask_t>>(g);
        auto &node = *m_delivery_task_node;
        {
            auto is_built = node.is_built();
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Frame delivery node is built: {}", is_built ? "true" : "false");
        }

        // set node params
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Setting node params ...");
        auto buffer_size = m_init_config.get_num_buffer_requests();
        node.set_input_data_buffer_size(buffer_size);
        node.set_preserve_order(m_init_config.get_preserve_request_order());

        // sync mode, all functions are executed in the graph
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Setting node to sync mode ...");
        node.set_use_async_callback(false);
        using DeliveryTaskNode_t = async_processor::SingleBufferExecNode<DeliveryTask_t>;
        using WorkInput_t = DeliveryTaskNode_t::InputWithTokens_t;
        using WorkOutput_t = DeliveryTaskNode_t::OutputWithTokens_t;

        // setup work function, nothing to do because during work function
        // frames are out of order
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Setting work function ...");
        node.set_work_function(
            [this](const WorkInput_t &input, WorkOutput_t &output) -> int {
                // copy input to output
                auto &out_payload = std::get<0>(output);
                const auto &in_payload = std::get<0>(input);
                auto ret = this->_do_task_delivery_preprocess(in_payload, out_payload);
                if (ret != 0) {
                    RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "Failed to preprocess frame, error code: {}", ret);
                }

                return ret;
            });

        // output callback
        // send frame to downstreams
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Setting output callback ...");
        node.set_output_callback(
            [this](const WorkOutput_t &output) -> int {
                auto &out_payload = std::get<0>(output);
                auto result = this->_do_task_delivery_main(out_payload);
                if (result.result_code != DeliveryResultCode::Success) {
                    RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "Failed to deliver frame, error code: {}", (int)result.result_code);
                }
                return static_cast<int>(result.result_code);
            });

        // build the node
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Building frame delivery node ...");
        node.build();

        return 0;
    }

    //! Connect to downstream nodes
    //! @return 0 if success, otherwise return error code
    virtual int _connect_to_downstreams()
    {

        // find and connect to downstreams
        m_downstreams.clear();
        for (auto &it : m_init_config.get_downstream_specs()) {
            Downstream_t ds;
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Initializing downstream ...");
            auto ret = ds.init_by_spec(it, m_parent_node);
            if (ret != 0) {
                RDX_RAISE_ERROR("[{}] failed to initialize downstream", __func__);
            }
            m_downstreams.push_back(ds);
        }

        return 0;
    }

    //! Set the status code of the port
    void _set_status_code(int status_code)
    {
        m_status = status_code;
    }

    virtual void _create_frame_delivery_task(const DeliveryRequest_t &request, DeliveryTask_t &task_output)
    {
        task_output.set_request(request);
    }


    /**
     * @brief Publish debug message when failed to send to downstream
     * @param source_data The source data to publish, if nullptr, no source data will be published
     * @param target_data The target data to publish, if nullptr, no target data will be published
     * @param ds The downstream node to publish to
     * @param ith_attempt Current attempt number
     * @param max_attempts Maximum number of attempts allowed
     * @return 0 on success, otherwise return error code
     */
    virtual int _debug_publish_failed_to_send_to_downstream(const SourceData_t *source_data,
                                                            const TargetData_t *target_data,
                                                            const Downstream_t &ds,
                                                            int64_t ith_attempt,
                                                            int64_t max_attempts)
    {
        if (!get_publish_to_debug_topic()) {
            return 0;
        }

        if (source_data != nullptr) {
            auto pub = ds.get_debug_pub_source_data_failed();
            if (pub != nullptr) {
                SourceData_t::PublishMessageType_t source_pub_msg;
                source_data->to_publish_message(source_pub_msg);
                auto s = fmt::format("[FAILED] attempt {}/{}", ith_attempt, max_attempts);
                pub->publish(source_pub_msg, s);
            }
        }
        if (target_data != nullptr) {
            auto pub = ds.get_debug_pub_target_data_failed();
            if (pub != nullptr) {
                TargetData_t::PublishMessageType_t target_pub_msg;
                target_data->to_publish_message(target_pub_msg);
                auto s = fmt::format("[FAILED] attempt {}/{}", ith_attempt, max_attempts);
                pub->publish(target_pub_msg, s);
            }
        }
        return 0;
    }

    /**
     * @brief Publish debug message when sending to downstream
     * @param source_data The source data to publish, if nullptr, no source data will be published
     * @param target_data The target data to publish, if nullptr, no target data will be published
     * @param ds The downstream node to publish to
     * @param ith_attempt Current attempt number
     * @param max_attempts Maximum number of attempts allowed
     * @return 0 on success, otherwise return error code
     */
    virtual int _debug_publish_sending_to_downstream(const SourceData_t *source_data,
                                                     const TargetData_t *target_data,
                                                     const Downstream_t &ds,
                                                     int64_t ith_attempt,
                                                     int64_t max_attempts)
    {
        if (!get_publish_to_debug_topic()) {
            return 0;
        }

        if (source_data != nullptr) {
            auto pub = ds.get_debug_pub_source_data_sending();
            if (pub != nullptr) {
                SourceData_t::PublishMessageType_t source_pub_msg;
                source_data->to_publish_message(source_pub_msg);
                auto s = fmt::format("[SENDING] attempt {}/{}", ith_attempt, max_attempts);
                pub->publish(source_pub_msg, s);
            }
        }
        if (target_data != nullptr) {
            auto pub = ds.get_debug_pub_target_data_sending();
            if (pub != nullptr) {
                TargetData_t::PublishMessageType_t target_pub_msg;
                target_data->to_publish_message(target_pub_msg);
                auto s = fmt::format("[SENDING] attempt {}/{}", ith_attempt, max_attempts);
                pub->publish(target_pub_msg, s);
            }
        }
        return 0;
    }

    /**
     * @brief Publish debug message when successfully sent to downstream
     * @param source_data The source data to publish, if nullptr, no source data will be published
     * @param target_data The target data to publish, if nullptr, no target data will be published
     * @param ds The downstream node to publish to
     * @param ith_attempt Current attempt number
     * @param max_attempts Maximum number of attempts allowed
     * @return 0 on success, otherwise return error code
     */
    virtual int _debug_publish_sent_to_downstream(const SourceData_t *source_data,
                                                  const TargetData_t *target_data,
                                                  const Downstream_t &ds,
                                                  int64_t ith_attempt,
                                                  int64_t max_attempts)
    {
        if (!get_publish_to_debug_topic()) {
            return 0;
        }

        if (source_data != nullptr) {
            auto pub = ds.get_debug_pub_source_data_succeeded();
            if (pub != nullptr) {
                SourceData_t::PublishMessageType_t source_pub_msg;
                source_data->to_publish_message(source_pub_msg);
                auto s = fmt::format("[SENT] attempt {}/{}", ith_attempt, max_attempts);
                pub->publish(source_pub_msg, s);
            }
        }
        if (target_data != nullptr) {
            auto pub = ds.get_debug_pub_target_data_succeeded();
            if (pub != nullptr) {
                TargetData_t::PublishMessageType_t target_pub_msg;
                target_data->to_publish_message(target_pub_msg);
                auto s = fmt::format("[SENT] attempt {}/{}", ith_attempt, max_attempts);
                pub->publish(target_pub_msg, s);
            }
        }
        return 0;
    }

    virtual int _create_target_data(TargetData_t &target_data, const DeliveryRequest_t &request)
    {
        target_data.set_source_data_uuid(request.get_source_data().get_uuid());
        return 0;
    }

    //! preprocess the delivery task, and transform it to the target data
    virtual int _do_task_delivery_preprocess(
        const DeliveryTask_t &task_input,
        DeliveryTask_t &task_output)
    {
        // nothing to do, just do shallow copy
        task_output = task_input;
        return 0;
    }

    //! main delivery logic
    //! @note request's precondition is checked before delivery happens, downstream precondition is ignored
    virtual DeliveryResult_t _do_task_delivery_main(const DeliveryTask_t &task)
    {
        // default precondition is any downstream ready
        auto request_precondition = m_init_config.get_fallback_delivery_precondition();

        // get the request's delivery policy, if any
        auto request_delivery_policy = task.get_request().get_delivery_policy();
        if (request_delivery_policy) {
            // use the request's delivery policy if it is set
            request_precondition = request_delivery_policy->get_precondition();
        }

        // check if precondition is satisfied
        bool precondition_satisfied = false;
        if (request_precondition == DeliveryPrecondition::DontCare || request_precondition == DeliveryPrecondition::NoPrecondition) {
            // if precondition is dont care or no precondition, it is regarded as satisfied
            precondition_satisfied = true;
        } else if (request_precondition == DeliveryPrecondition::AnyDownstreamReady || request_precondition == DeliveryPrecondition::AllDownstreamsReady) {
            // test for precondition: any downstream ready or all downstreams ready

            // count the number of downstreams that are ready, apply precondition
            size_t num_downstream_ready = 0;
            for (auto &ds : m_downstreams) {
                bool ds_ready = false;

                // when testing precondition, we use the downstream's delivery policy
                auto &ds_policy = ds.get_downstream_spec().get_delivery_policy();

                // check if downstream is ready
                auto &retry_policy = ds_policy.get_retry_policy();
                auto fallback_wait_time = retry_policy.get_fallback_wait_time_retry_response();
                auto ping_wait_time = retry_policy.get_wait_time_retry_response().value_or(fallback_wait_time);
                ds_ready = _ping(ds, ping_wait_time);

                // count the number of downstreams that are ready
                num_downstream_ready += ds_ready ? 1 : 0;

                // if any downstream is ready, break
                if (request_precondition == DeliveryPrecondition::AnyDownstreamReady) {
                    if (num_downstream_ready > 0) {
                        precondition_satisfied = true;
                        break;
                    }
                } else if (request_precondition == DeliveryPrecondition::AllDownstreamsReady) {
                    if (num_downstream_ready == m_downstreams.size()) {
                        precondition_satisfied = true;
                        break;
                    }
                }
            }
        }

        if (!precondition_satisfied) {
            RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "Precondition is not satisfied");
            return DeliveryResult_t{DeliveryResultCode::NotTried};
        }

        RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "Precondition is satisfied, start delivery ...");

        // create target delivery data
        TargetData_t target_data;
        _create_target_data(target_data, task.get_request());

        // do whatever preprocessing you need
        if (m_cb_on_deliver_before) {
            auto ret = m_cb_on_deliver_before(target_data, task);
            if (ret != 0) {
                RDX_LOG_WARN(m_parent_node, __func__, PRINT_THREAD_ID, "on_deliver_before callback failed");
            }
        }

        //! Deliver data
        DeliveryResult_t result;
        {
            bool delivered_to_any_downstream = false;

            // deliver to all downstreams, if any delivery succeeds, the result is regarded as success
            for (auto &ds : m_downstreams) {
                auto ret = _deliver_data_with_retry(target_data, ds, request_delivery_policy);
                if (ret != 0) {
                    RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID,
                                 "Failed to deliver frame to downstream {}",
                                 ds.get_downstream_spec().get_name());
                } else {
                    delivered_to_any_downstream = true;
                }
            }

            if (!delivered_to_any_downstream) {
                RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "Failed to deliver data to any downstream");
                result = DeliveryResult_t{DeliveryResultCode::TriedButFailed};
            } else {
                RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "Successfully delivered data to some downstreams");
                result = DeliveryResult_t{DeliveryResultCode::Success};
            }
        }

        // do whatever postprocessing you need
        if (m_cb_on_deliver_after) {
            auto ret = m_cb_on_deliver_after(target_data, task, result);
            if (ret != 0) {
                RDX_LOG_WARN(m_parent_node, __func__, PRINT_THREAD_ID, "on_deliver_after callback failed");
            }
        }

        return result;
    }

    //! delivery data to downstream, with retry logic
    virtual int
        _deliver_data_with_retry(const TargetData_t &target_data,
                                 const Downstream_t &ds,
                                 const DeliveryPolicy_t *prefer_delivery_policy = nullptr)
    {
        //! Set up retry strategy
        int attempts = 0;

        //! use the preferred delivery policy if provided, otherwise use the downstream's delivery policy
        const DeliveryPolicy_t *delivery_policy = nullptr;
        if (prefer_delivery_policy != nullptr) {
            delivery_policy = prefer_delivery_policy;
        } else {
            delivery_policy = &ds.get_downstream_spec().get_delivery_policy();
        }
        const auto &retry_policy = delivery_policy->get_retry_policy();

        // get the retry parameters
        bool no_drop = delivery_policy->get_drop_strategy() == DropStrategy::NoDrop;
        auto max_attempts = retry_policy.get_number_of_retry(true).value() + 1; // +1 for the first attempt
        auto timeout_each_attempt = retry_policy.get_wait_time_retry_response(true).value();
        auto interval_between_attempts = retry_policy.get_wait_time_between_retry(true).value();
        auto msg_uuid = target_data.get_source_data_uuid();

        // deliver until max attempts reached, or until success when no drop is required
        while (attempts < max_attempts || no_drop) {
            if (!rclcpp::ok()) {
                // system is shutting down, get out
                return -1;
            }

            // for no_drop, we need to keep trying until the frame is delivered (return in the loop)
            //! Publish the frame to the debug topic
            _debug_publish_sending_to_downstream(nullptr, &target_data, ds, attempts + 1, max_attempts);

            //! Send the frame to the downstream
            auto result = _send_data_to_downstream(target_data, ds, timeout_each_attempt);
            if (!result.goal_handle_future.valid()) {
                RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Not sending frame to downstream {}, goal handle future is invalid",
                             boost::uuids::to_string(msg_uuid), ds.get_downstream_spec().get_name());
            } else {
                bool wait_indefinitely = timeout_each_attempt < DefaultTimeUnit_t::zero();

                if (result.response_code.has_value()) {
                    // it has a response code, check it
                    switch (*result.response_code) {
                        case ActionDownstreamResponse::ACCEPTED:
                            RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Frame accepted by downstream {}",
                                         boost::uuids::to_string(msg_uuid), ds.get_downstream_spec().get_name());

                            //! Publish the frame sent message
                            _debug_publish_sent_to_downstream(nullptr, &target_data, ds, attempts + 1, max_attempts);

                            return 0; // Success
                        case ActionDownstreamResponse::REJECTED:
                            RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Frame rejected by downstream {}",
                                         boost::uuids::to_string(msg_uuid), ds.get_downstream_spec().get_name());
                            break;
                        case ActionDownstreamResponse::TIMEOUT:
                            RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Timeout while sending frame to downstream {}",
                                         boost::uuids::to_string(msg_uuid), ds.get_downstream_spec().get_name());
                            break;
                    }
                } else {
                    // may or maynot have a response code, check the goal handle future
                    if (wait_indefinitely) {
                        //! Wait indefinitely for the goal handle future
                        result.goal_handle_future.wait();
                        auto goal_handle = result.goal_handle_future.get();
                        if (goal_handle) {
                            return 0; // Success
                        }
                    } else {
                        //! Regard as failure without additional waiting
                        RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Timeout while waiting for goal handle from downstream {}",
                                     boost::uuids::to_string(msg_uuid), ds.get_downstream_spec().get_name());
                    }
                }
            }

            attempts++;
            if (attempts < max_attempts) {
                RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Retrying frame delivery to downstream {} (attempt {}/{})",
                             boost::uuids::to_string(msg_uuid), ds.get_downstream_spec().get_name(), attempts + 1, max_attempts);
            }

            // sleep for the interval between attempts
            std::this_thread::sleep_for(interval_between_attempts);
        }

        _debug_publish_failed_to_send_to_downstream(nullptr, &target_data, ds, attempts, max_attempts);

        RDX_INFO_DEV(m_parent_node, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Failed to deliver frame to downstream {} after {} attempts",
                     boost::uuids::to_string(msg_uuid), ds.get_downstream_spec().get_name(), max_attempts);
        return -1;
    }

    //! send a single piece of data to downstream
    virtual SendResult_t _send_data_to_downstream(const TargetData_t &target_data,
                                                  const Downstream_t &ds,
                                                  TimeUnit_t timeout)
    {
        //! Get the action client for the downstream
        auto client = ds.get_action_client();

        //! Create a goal object and populate it with frame message data
        auto &goal = target_data.get_goal();

        //! Use SyncActionSender to send the goal and wait for the response
        SyncActionSender_t sender(m_parent_node);
        auto result = sender.send(goal, *client, timeout);

        return result;
    }

    //! Ping the downstream node
    //! @return true if success, otherwise false
    virtual bool _ping(const Downstream_t &ds, TimeUnit_t timeout)
    {
        //! Create a target data object for pinging
        TargetData_t target_data;
        target_data.set_source_data_uuid(boost::uuids::random_generator()());
        ActionDataTrait_t::mark_with_control_signal(target_data.get_goal(), ControlSignalCode::Ping);

        //! Send the ping message to the downstream
        auto result = _send_data_to_downstream(target_data, ds, timeout);

        //! Check the response
        if (timeout == DefaultTimeUnit_t(0)) {
            //! If timeout is 0, return false as we cannot get a response in no time
            return false;
        } else if (timeout > DefaultTimeUnit_t(0)) {
            //! If timeout is positive, check the response code
            //! Anything other than ACCEPTED is considered not ready
            return result.response_code.has_value() &&
                   result.response_code.value() == ActionDownstreamResponse::ACCEPTED;
        } else {
            //! If timeout is negative, wait indefinitely for the goal handle future
            result.goal_handle_future.wait();
            auto goal_handle = result.goal_handle_future.get();
            return goal_handle != nullptr;
        }
    }

  protected:
    // the status of the port, can be one of the following:
    // BEFORE_INIT, STARTED, STOPPED
    std::atomic<int> m_status = NodeStatusCode::BEFORE_INIT;

    // publish to debug topics?
    std::atomic<bool> m_publish_to_debug_topic = false;

    // init config
    InitConfig_t m_init_config;

    // downstreams
    std::vector<Downstream_t> m_downstreams;

    // the parent node
    rclcpp::Node *m_parent_node = nullptr;


  protected:
    // callback functions

    //! Transform target data, (output_target_data, input_task)-> error_code
    std::function<int(TargetData_t &output,
                      const DeliveryTask_t &input)>
        m_cb_on_deliver_before;

    //! Clean target data, (output_target_data, input_task)-> error_code
    std::function<int(TargetData_t &output,
                      const DeliveryTask_t &input,
                      const DeliveryResult_t &result)>
        m_cb_on_deliver_after;

  private:
    //! used tbb to process the delivery tasks
    std::shared_ptr<DeliveryTaskNode_t> m_delivery_task_node;
    std::shared_ptr<tbb::flow::graph> m_delivery_graph;
    tbb::task_group m_task_group; // all async tasks
};


} // namespace redoxi_works