#pragma once

#include <functional>
#include <atomic>
#include <tbb/concurrent_hash_map.h>
#include <tbb/concurrent_queue.h>
#include <boost/uuid/uuid_io.hpp>
#include <boost/thread/synchronized_value.hpp>

#include <redoxi_common_nodes/async_action_port/input_port_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionInputTypes.hpp>
#include <redoxi_common_cpp/redoxi_tbb_util.hpp>

// #define ASYNC_INPUT_PORT_USE_TBB_HASH_MAP

namespace redoxi_works
{
using TSpec = input_port_types::AsyncActionInputPortSpec;

class AsyncActionInputPort : public IStartStopProtocol
{
  private:
    inline static constexpr auto PRINT_THREAD_ID = false;

    // initial size of the source data map, must be large enough to avoid rehashing
    inline static constexpr auto InitialSourceDataMapSize = 10000;
    inline static constexpr auto GoalHandleTimeout = DefaultParams::GoalHandleTimeout;

  public:
    AsyncActionInputPort(rclcpp::Node *parent_node)
        : m_parent_node(parent_node)
    {
        m_ping_response = std::make_shared<ActionResult_t>();
    }

    virtual ~AsyncActionInputPort() noexcept = default;

    // useful types
    using MasterSpec_t = TSpec; // master specification of this port
    using ActionType_t = TSpec::ActionType_t;
    using ActionGoal_t = TSpec::ActionGoal_t;
    using ActionResult_t = ActionType_t::Result;
    using ActionFeedback_t = ActionType_t::Feedback;
    using ActionDataTrait_t = TSpec::ActionDataTrait_t;
    using InitConfig_t = TSpec::InitConfig_t;
    using SourceData_t = TSpec::ReceiveSourceData_t;
    using GoalHandle_t = SourceData_t::GoalHandle_t;
    using GoalUUID_t = rclcpp_action::GoalUUID;

  public:
    //! Initialize the port, state transition: BEFORE_INIT -> STOPPED
    int init(std::shared_ptr<InitConfig_t> config)
    {
        // assert status must be before init
        if (m_status != NodeStatusCode::BEFORE_INIT) {
            RDX_RAISE_ERROR("[{}] Port is not in the correct status, got {}", __func__, NodeStatusCodeToString(m_status));
            return -1;
        }

        m_init_config = config;

        // create the action server
        auto action_name = m_init_config->get_action_name();
        if (action_name.empty()) {
            RDX_RAISE_ERROR("[{}] Action name is empty", __func__);
            return -1;
        }

        // create the action server
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "Creating action server [{}]", action_name);

        // override the default result timeout, otherwise there may be too many pending goals
        rcl_action_server_options_t server_options = rcl_action_server_get_default_options();
        std::chrono::nanoseconds timeout_ns = GoalHandleTimeout;
        server_options.result_timeout.nanoseconds = timeout_ns.count();

        m_action_server = rclcpp_action::create_server<ActionType_t>(
            m_parent_node, action_name,
            std::bind(&AsyncActionInputPort::_on_goal_received, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AsyncActionInputPort::_on_goal_cancel_request, this, std::placeholders::_1),
            std::bind(&AsyncActionInputPort::_on_goal_accepted, this, std::placeholders::_1),
            server_options);

        // initialize the queue
        int64_t buffer_capacity = m_init_config->get_buffer_capacity();
        if (buffer_capacity > 0) {
            m_source_data_queue.set_capacity(buffer_capacity);
        }

        // state transition: BEFORE_INIT -> STOPPED
        _set_status(NodeStatusCode::STOPPED);
        return 0;
    }

    //! Start the port, state transition: STOPPED -> STARTED
    //! @note the port will start receiving goals, otherwise it will reject all goals
    //! @return 0 if success, otherwise error
    int start() override
    {
        // already started?
        if (m_status == NodeStatusCode::STARTED) {
            return 0;
        }

        // assert status must be stopped
        if (m_status != NodeStatusCode::STOPPED) {
            RDX_RAISE_ERROR("[{}] Port is not in the correct status, got {}", __func__, NodeStatusCodeToString(m_status));
        }

        // state transition: STOPPED -> STARTED
        _set_status(NodeStatusCode::STARTED);

        return 0;
    }

    //! Stop the port, state transition: STARTED -> STOPPED
    //! @note the port will reject all goals, but previously accepted goals will still be processed
    //! @return 0 if success, otherwise error
    int stop() override
    {
        // already stopped?
        if (m_status == NodeStatusCode::STOPPED) {
            return 0;
        }

        // assert status must be started
        if (m_status != NodeStatusCode::STARTED) {
            RDX_RAISE_ERROR("[{}] Port is not in the correct status, got {}", __func__, NodeStatusCodeToString(m_status));
        }

        // reset the queue
        _reset_queue();

        // state transition: STARTED -> STOPPED
        _set_status(NodeStatusCode::STOPPED);
        return 0;
    }

    //! Set the buffer size, negative value means unbounded
    //! @note you should only call this when stopped
    void set_buffer_capacity(int64_t size)
    {
        // assert status must be stopped
        if (m_status != NodeStatusCode::STOPPED) {
            RDX_RAISE_ERROR("[{}] Port is not in the correct status, got {}", __func__, NodeStatusCodeToString(m_status));
        }

        m_source_data_queue.set_capacity(size);
    }

    //! Get the buffer capacity
    int64_t get_buffer_capacity() const
    {
        return m_source_data_queue.capacity();
    }

    //! Get the number of buffered requests
    int64_t get_num_buffered_requests() const
    {
        return m_source_data_queue.size();
    }

    //! try pop a source data from the queue, return nullptr if no data
    std::shared_ptr<SourceData_t> try_pop_source_data()
    {
        std::shared_ptr<SourceData_t> data;
        bool success = m_source_data_queue.try_pop(data);
        return success ? data : nullptr;
    }

    //! pop a source data and wait until one is available
    std::shared_ptr<SourceData_t> pop_source_data()
    {
        std::shared_ptr<SourceData_t> data;
        m_source_data_queue.pop(data);
        return data;
    }

  protected:
    //! Reset the queue, clear all data and wake up all waiting threads
    void _reset_queue()
    {
        // wake up all waiting threads
        m_source_data_queue.abort();

        // pop all data at once until the queue is empty
        std::shared_ptr<SourceData_t> data;
        while (m_source_data_queue.try_pop(data)) {
        }
    }

    //! Set the status
    void _set_status(int status)
    {
        m_status = status;
    }

    //! Handle goal received
    virtual rclcpp_action::GoalResponse _on_goal_received(const rclcpp_action::GoalUUID &uuid,
                                                          std::shared_ptr<const ActionGoal_t> goal)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        const auto msg_uuid = ActionDataTrait_t::get_uuid(*goal);
        const auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
        const bool is_ping = ActionDataTrait_t::get_control_signal_code(*goal) == ControlSignalCode::Ping;
        const std::string log_prefix = "[msg_uuid=" + msg_uuid_str + "]";

        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} Goal received, is_ping={}", log_prefix, is_ping);

        // if not started, reject everything
        if (m_status != NodeStatusCode::STARTED) {
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} REJECTED, Port is not started", log_prefix);
            return rclcpp_action::GoalResponse::REJECT;
        }

        // handle ping signal
        if (is_ping) {
            bool has_room = m_source_data_queue.size() < m_source_data_queue.capacity();
            if (has_room) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            } else {
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // non ping, do the normal processing

        // notify user first
        if (m_on_goal_received_callback) {
            int ret = m_on_goal_received_callback(uuid, goal);
            if (ret != 0) {
                RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} REJECTED, User does not want to accept the goal", log_prefix);
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // create source data
        auto source_data = std::make_shared<SourceData_t>();

        // must fill the following before pushing to the queue
        // otherwise, it may get dequeued by other threads and processed while the goal is not set
        source_data->set_goal(goal);
        source_data->set_goal_uuid(uuid);
        source_data->set_goal_handle_promise(std::make_shared<SourceData_t::GoalHandlePromise_t>());

        // can we push it to the queue?
        bool enqueued = m_source_data_queue.try_push(source_data);
        if (enqueued) {
            // enqueued successfully, set the goal data, and bookkeep the promise
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} ACCEPTED, Goal enqueued, num goals in the queue={}", log_prefix, m_source_data_queue.size());

            // save to map
            auto key = to_boost_uuid(uuid);
#ifdef ASYNC_INPUT_PORT_USE_TBB_HASH_MAP
            SourceDataMap_t::accessor acc;
            bool inserted_as_new = m_source_data_map.insert(acc, key);
            if (!inserted_as_new) {
                RDX_RAISE_ERROR("{} Failed to insert goal to map", log_prefix);
            }
            acc->second = source_data;
#else
            m_source_data_map->insert({key, source_data});
#endif

            // notify user
            if (m_on_goal_enqueued_callback) {
                m_on_goal_enqueued_callback(source_data);
            }

            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} ACCEPTED, Goal enqueued, waiting for goal handle", log_prefix);

            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            RDX_INFO_DEV(m_parent_node, __func__, true, "[msg_uuid={}] goal enqueue time: {} ms", boost::uuids::to_string(msg_uuid),
                         elapsed_time.count());

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            // failed to push to queue, reject the goal
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} REJECTED, Failed to enqueue goal", log_prefix);
            if (m_on_goal_rejected_callback) {
                m_on_goal_rejected_callback(uuid, goal);
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            RDX_INFO_DEV(m_parent_node, __func__, true, "[msg_uuid={}] goal enqueue time: {} ms", boost::uuids::to_string(msg_uuid),
                         elapsed_time.count());

            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    //! Handle cancel request
    virtual rclcpp_action::CancelResponse _on_goal_cancel_request(std::shared_ptr<GoalHandle_t> goal_handle)
    {
        const auto msg_uuid = ActionDataTrait_t::get_uuid(*goal_handle->get_goal());
        const auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
        const bool is_ping = ActionDataTrait_t::get_control_signal_code(*goal_handle->get_goal()) == ControlSignalCode::Ping;
        const std::string log_prefix = "[msg_uuid=" + msg_uuid_str + "]";

        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} Cancel request received, is_ping={}", log_prefix, is_ping);

        if (is_ping) {
            // ping signal, do nothing
            goal_handle->canceled(m_ping_response);
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // notify user
        if (m_on_goal_cancel_request_callback) {
            int ret = m_on_goal_cancel_request_callback(goal_handle);
            if (ret != 0) {
                // user does not want to cancel the goal
                RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} REJECTED, User does not want to cancel the goal", log_prefix);
                return rclcpp_action::CancelResponse::REJECT;
            }
        }

        // signal the goal handle promise
        _resolve_source_data(goal_handle);

        // always accept cancel request
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} CANCELLED, removed goal from map", log_prefix);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    //! Handle accepted goal
    virtual void _on_goal_accepted(std::shared_ptr<GoalHandle_t> goal_handle)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        const auto msg_uuid = ActionDataTrait_t::get_uuid(*goal_handle->get_goal());
        const auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
        const bool is_ping = ActionDataTrait_t::get_control_signal_code(*goal_handle->get_goal()) == ControlSignalCode::Ping;
        const std::string log_prefix = "[msg_uuid=" + msg_uuid_str + "]";

        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} Goal accepted, is_ping={}", log_prefix, is_ping);
        if (is_ping) {
            // ping signal, do nothing
            // goal_handle->succeed(m_ping_response);
            // use abort to terminate the goal immediately
            goal_handle->abort(m_ping_response);
            return;
        }

        // notify user
        if (m_on_goal_accepted_callback) {
            m_on_goal_accepted_callback(goal_handle);
        }

        _resolve_source_data(goal_handle);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        RDX_INFO_DEV(m_parent_node, __func__, true, "[msg_uuid={}] goal accepted time: {} ms", boost::uuids::to_string(msg_uuid),
                     elapsed_time.count());

        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} ACCEPTED, goal promise set, removed goal from map", log_prefix);
    }

#ifndef ASYNC_INPUT_PORT_USE_TBB_HASH_MAP
    virtual void _resolve_source_data(std::shared_ptr<GoalHandle_t> goal_handle)
    {
        const auto key = to_boost_uuid(goal_handle->get_goal_id());
        const auto key_str = boost::uuids::to_string(key);
        const std::string log_prefix = "[msg_uuid=" + key_str + "]";

        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID,
                      "{} Resolving goal handle", log_prefix);

        // lock the map
        auto locked_map = m_source_data_map.synchronize();

        // look for the goal in the map
        auto it = locked_map->find(key);
        if (it == locked_map->end()) {
            // may have been cancelled, just return
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} COMPLETED, though goal is not in the map", log_prefix);
            return;
        }

        // set the result
        auto p = it->second->get_goal_handle_promise();
        if (p) {
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} COMPLETED, setting promise to wake up waiting threads", log_prefix);
            try {
                p->set_value(goal_handle);
            } catch (const std::future_error &e) {
                if (e.code() != std::future_errc::promise_already_satisfied) {
                    throw; // rethrow if it's not the promise already satisfied error
                }
                // Otherwise, ignore the exception as the promise is already set
            }
        }

        // remove the goal from the map
        locked_map->erase(it);
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} COMPLETED, num of unresolved goals={}", log_prefix, locked_map->size());
    }
#else
    virtual void _resolve_source_data(std::shared_ptr<GoalHandle_t> goal_handle)
    {
        // get from the map
        SourceDataMap_t::accessor acc;
        const auto key = to_boost_uuid(goal_handle->get_goal_id());
        const auto key_str = boost::uuids::to_string(key);
        const std::string log_prefix = "[msg_uuid=" + key_str + "]";

        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID,
                      "{} Resolving goal handle", log_prefix);

        bool found = m_source_data_map.find(acc, key);
        if (!found) {
            // may have been cancelled, just return
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} COMPLETED, though goal is not in the map", log_prefix);
            return;
        }

        // set the result
        auto p = acc->second->get_goal_handle_promise();
        if (p) {
            RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} COMPLETED, setting promise to wake up waiting threads", log_prefix);
            try {
                p->set_value(goal_handle);
            } catch (const std::future_error &e) {
                if (e.code() != std::future_errc::promise_already_satisfied) {
                    throw; // rethrow if it's not the promise already satisfied error
                }
                // Otherwise, ignore the exception as the promise is already set
            }
        }

        // remove the goal from the map
        m_source_data_map.erase(acc);
        RDX_LOG_DEBUG(m_parent_node, __func__, PRINT_THREAD_ID, "{} COMPLETED, num of unresolved goals={}", log_prefix, m_source_data_map.size());
    }
#endif

  protected:
    rclcpp::Node *m_parent_node = nullptr;
    std::shared_ptr<InitConfig_t> m_init_config;

    // the action server
    rclcpp_action::Server<ActionType_t>::SharedPtr m_action_server;

#ifdef ASYNC_INPUT_PORT_USE_TBB_HASH_MAP
    // buffer map for storing source data, unordered
    using SourceDataMap_t = tbb::concurrent_hash_map<boost::uuids::uuid,
                                                     std::shared_ptr<SourceData_t>,
                                                     TbbBoostUuidHash>;
    SourceDataMap_t m_source_data_map{InitialSourceDataMapSize};
#else
    using SourceDataMap_t = boost::synchronized_value<std::map<boost::uuids::uuid, std::shared_ptr<SourceData_t>>>;
    SourceDataMap_t m_source_data_map;
#endif
    // queue for storing source data, ordered
    using SourceDataQueue_t = tbb::concurrent_bounded_queue<std::shared_ptr<SourceData_t>>;
    SourceDataQueue_t m_source_data_queue;


  protected:
    // callbacks, note that these callbacks do not receive ping signal

    //! Callback for goal received, return 0 if accepted, otherwise rejected
    std::function<int(const GoalUUID_t &, std::shared_ptr<const ActionGoal_t>)> m_on_goal_received_callback;

    //! Callback for goal enqueued
    //! This happens when the goal is accepted but goal handle is not created yet
    std::function<void(std::shared_ptr<SourceData_t>)> m_on_goal_enqueued_callback;

    //! Callback for goal rejection
    //! This happens when the goal is rejected by the port for any reason
    std::function<void(const GoalUUID_t &, std::shared_ptr<const ActionGoal_t>)> m_on_goal_rejected_callback;

    //! Callback for goal cancel request, return 0 if accepted, otherwise rejected
    std::function<int(std::shared_ptr<GoalHandle_t>)> m_on_goal_cancel_request_callback;

    //! Callback for goal accepted
    std::function<void(std::shared_ptr<GoalHandle_t>)> m_on_goal_accepted_callback;

  public:
    //! Set callback for goal received
    //! @param callback the callback function, return 0 if accepted, otherwise rejected
    void set_on_goal_received_callback(std::function<int(const GoalUUID_t &, std::shared_ptr<const ActionGoal_t>)> callback)
    {
        m_on_goal_received_callback = callback;
    }

    //! Set callback for goal enqueued
    void set_on_goal_enqueued_callback(std::function<void(std::shared_ptr<SourceData_t>)> callback)
    {
        m_on_goal_enqueued_callback = callback;
    }

    //! Set callback for goal rejection
    void set_on_goal_rejected_callback(std::function<void(const GoalUUID_t &, std::shared_ptr<const ActionGoal_t>)> callback)
    {
        m_on_goal_rejected_callback = callback;
    }

    //! Set callback for goal cancel request
    //! @param callback the callback function, return 0 if accepted, otherwise rejected
    void set_on_goal_cancel_request_callback(std::function<int(std::shared_ptr<GoalHandle_t>)> callback)
    {
        m_on_goal_cancel_request_callback = callback;
    }

    //! Set callback for goal accepted
    void set_on_goal_accepted_callback(std::function<void(std::shared_ptr<GoalHandle_t>)> callback)
    {
        m_on_goal_accepted_callback = callback;
    }

  private:
    // cache ping response
    std::shared_ptr<ActionResult_t>
        m_ping_response;

    // status
    std::atomic<int> m_status = NodeStatusCode::BEFORE_INIT;
};

} // namespace redoxi_works
