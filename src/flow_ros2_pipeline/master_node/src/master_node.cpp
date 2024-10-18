#include "psg_common/psg_common.hpp"
#include <boost/thread/lock_algorithms.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <set>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rcpputils/asserts.hpp>

#include <master_node/_master_node.hpp>
#include <master_node/master_node.hpp>
#include <vector>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

#define COMPILE_MASTER_NODE
#ifdef COMPILE_MASTER_NODE
namespace FlowRos2Pipeline
{
MasterNode::MasterNode()
    : Node("master_node")
{
    m_impl = std::make_shared<MasterNodeImpl>(this);

    _declare_all_parameters();

    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    // m_impl->sync_document_doing_map = &m_psgdoc_task_doing;
    m_impl->sync_frame_buffer = &m_frame_buffer;

    m_memory_registry = std::make_shared<MemoryRegistry>(this);

    // RCLCPP_INFO(m_impl->logger, "[MasterNode] constraction success!");
}

void MasterNode::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_frame_action", "");
    this->declare_parameter<std::string>("downstream_action", "");
    this->declare_parameter<std::string>("status_query_service", "");
    this->declare_parameter<std::string>("send_frame_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", 10000);
    this->declare_parameter<int>("buffer_size", 1);
    this->declare_parameter<bool>("send_goal_retry", false);
}

void MasterNode::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "[MasterNode] m_init_config is nullptr");

    m_downstreams.clear();
    for (auto it : m_init_config->downstreams) {
        auto ds = std::make_shared<Downstream>();
        // RCLCPP_INFO(m_impl->logger, "[MasterNode] connecting to downstream %s", it.first.c_str());

        // 创建accept_frame_client
        {
            std::string name = it.second.accept_document_action;
            auto client = rclcpp_action::create_client<ACT_AcceptDocument>(this, name);

            ds->handler = client;
            ds->options.goal_response_callback =
                std::bind(&MasterNode::_process_document_goal_response_callback, this, std::placeholders::_1);
            ds->options.feedback_callback =
                std::bind(&MasterNode::_process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            ds->options.result_callback =
                std::bind(&MasterNode::_process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            // RCLCPP_INFO(m_impl->logger, "[MasterNode] waiting for action server %s", name.c_str());
            client->wait_for_action_server();
            // RCLCPP_INFO(m_impl->logger, "[MasterNode] action server %s is ready", name.c_str());
        }

        m_downstreams[it.first] = ds;
    }
}

int MasterNode::init(const std::shared_ptr<InitConfig> &config,
                     const std::shared_ptr<RuntimeConfig> &runtime_config)
{
    ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT || m_status_code == NodeStatusCode::STOPPED,
               "[MasterNode] init FAILED! status code is not BEFORE_INIT or STOPPED");

    m_init_config = config;
    m_runtime_config = runtime_config;

    m_memory_registry->connect_to_v6d("/var/run/vineyard.sock");

    // create status_query server
    m_srv_status_query = this->create_service<SRV_StatusQuery>(
        m_init_config->status_query_service, std::bind(&MasterNode::_status_query_callback, this, std::placeholders::_1, std::placeholders::_2));

    // setup downstreams
    _connect_to_downstreams();
    // create process_frame server
    m_act_accept_frame = rclcpp_action::create_server<ACT_AcceptFrame>(
        this, m_init_config->process_frame_action,
        std::bind(&MasterNode::_accept_frame_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MasterNode::_accept_frame_cancel_callback, this, std::placeholders::_1),
        std::bind(&MasterNode::_accept_frame_accepted_callback, this, std::placeholders::_1));
    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}
const std::shared_ptr<MasterNode::InitConfig> &MasterNode::get_init_config() const
{
    return m_init_config;
}
const std::shared_ptr<MasterNode::RuntimeConfig> &MasterNode::get_runtime_config() const
{
    return m_runtime_config;
}
int MasterNode::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");
    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}
int MasterNode::get_status_code() const
{
    return m_status_code;
}

void MasterNode::_process_document_goal_response_callback(
    const rclcpp_action::ClientGoalHandle<MasterNode::ACT_AcceptDocument>::SharedPtr &goal_handle)
{
    if (goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
        // TODO: here
    } else {
        // RCLCPP_INFO(m_impl->logger, "[MasterNode] Frame rejected");
    }
}

void MasterNode::_process_document_feedback_callback(rclcpp_action::ClientGoalHandle<MasterNode::ACT_AcceptDocument>::SharedPtr,
                                                     const std::shared_ptr<const MasterNode::ACT_AcceptDocument::Feedback> feedback)
{
    (void)feedback;
}

void MasterNode::_process_document_result_callback(
    const rclcpp_action::ClientGoalHandle<MasterNode::ACT_AcceptDocument>::WrappedResult &result)
{
    (void)result;
}


void MasterNode::_status_query_callback(const std::shared_ptr<SRV_StatusQuery::Request> request,
                                        std::shared_ptr<SRV_StatusQuery::Response> response)
{
    (void)request; // not used

    // if buffer is full, reject the frame
    if (m_frame_buffer.size() >= m_runtime_config->buffer_size) {
        response->status = ReturnCode::REJECTED;
        RCLCPP_INFO(m_impl->logger, "_status_query_callback() REJECTED, buffer size is %d", m_frame_buffer.size());
        return;
    }

    // RCLCPP_DEBUG(m_impl->logger, "Received status query request");
    response->status = ReturnCode::SUCCESS;
    RCLCPP_INFO(m_impl->logger, "_status_query_callback() SUCCESS, buffer size is %d", m_frame_buffer.size());
    // RCLCPP_DEBUG(m_impl->logger, "Response status query request");
}

rclcpp_action::GoalResponse MasterNode::_accept_frame_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptFrame::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with frame %ld", goal->frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MasterNode::_accept_frame_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void MasterNode::_accept_frame_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle)
{
    const auto &goal = goal_handle->get_goal();
    const auto &x_control = goal->x_control;

    RCLCPP_INFO(m_impl->logger, "frame %d m_frame_buffer buffer size: %d", goal->frame.frame_num, m_frame_buffer.size());

    // if buffer is full, reject the frame
    // 只有丢帧模式（不尝试重复发送），才会在这里被拦截，避免buffer溢出
    if (!m_runtime_config->send_goal_retry && m_frame_buffer.size() >= m_runtime_config->buffer_size) {
        auto result = std::make_shared<ACT_AcceptFrame::Result>();
        result->return_msg = "Buffer is full";
        result->return_code = ReturnCode::REJECTED;
        goal_handle->abort(result);
        RCLCPP_INFO(m_impl->logger, "REJECTED!!! Buffer is full");
        return;
    }

    // 当没有reject时，ping一定成功
    if (x_control.code == 1) {
        auto result = std::make_shared<ACT_AcceptFrame::Result>();
        result->return_msg = "Ping accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
        return;
    }

    // time log
    RCLCPP_INFO(m_impl->logger, "---TIME LOG: framenum %ld node %s type %s time %ld", goal->frame.frame_num, "master_node", "IN", this->now().nanoseconds());

    // cache the frame
    const auto &frame = goal->frame;

    // add to memory registry
    if (frame.signal_code == SignalCode::RUN) {
        auto lock_ptr_frame_buffer = m_impl->sync_frame_buffer.synchronize();
        _add_frame_to_buffer(frame, *lock_ptr_frame_buffer);
    }

    // RCLCPP_INFO(m_impl->logger, "Accepted frame %ld and add it to buffer", frame.frame_num);

    // create tasks for all downstreams
    {
        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        _process_document_create_tasks(frame, *lock_ptr_document_task_waiting);
    }

    auto result = std::make_shared<ACT_AcceptFrame::Result>();
    result->return_msg = "Frame accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
    RCLCPP_INFO(m_impl->logger, "Accepted frame %ld and set STATUS_SUCCEED", frame.frame_num);
}

void MasterNode::_add_frame_to_buffer(const MSG_Frame &frame, std::map<int, MasterNode::MSG_Frame> *frame_buffer_ptr)
{
    (*frame_buffer_ptr)[frame.frame_num] = frame;

    // add to memory registry
    MemoryEntry e;
    e.frame_number = frame.frame_num;
    e.name = "frame";

    ROS_ASSERT(frame.cache.has_int_id, "frame.cache has no int_id");

    e.v6d_object_id = frame.cache.id_int;
    m_memory_registry->add_entry(e);
}

void MasterNode::_remove_frame_from_buffer(int frame_number, std::map<int, MSG_Frame> *frame_buffer_ptr, bool remove_memory_entry)
{
    frame_buffer_ptr->erase(frame_number);
    if (remove_memory_entry)
        m_memory_registry->remove_entries_by_frame(frame_number);
}

void MasterNode::_process_document_create_tasks(const MSG_Frame &frame, Map_Document_Waiting *document_waiting_map_ptr)
{
    // create tasks of this frame for all downstreams
    for (auto &x : m_downstreams) {
        auto task = std::make_shared<DSTask_PSGDocument>();
        task->downstream = x.second;
        task->frame = frame;
        (*document_waiting_map_ptr)[std::make_tuple(task->downstream.get(), frame.frame_num)] = task;
    }
    RCLCPP_DEBUG(m_impl->logger, "_process_document_create_tasks: %ld, end", frame.frame_num);
}

int MasterNode::start()
{
    // the node must be opened
    ROS_ASSERT(m_status_code == NodeStatusCode::INITIALIZED,
               "cannot start because status code is not INITIALIZED");

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STARTED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);

    m_impl->step_running = true;
    m_impl->step_thread = std::make_shared<std::thread>(
        [this]() {
            while (rclcpp::ok() && m_impl->step_running) {
                _step();
                // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
            }
        });


    // auto step_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)),
    //                                           std::bind(&MasterNode::_step, this));
    // m_impl->step_timer = step_timer;

    return ReturnCode::SUCCESS;
}

int MasterNode::stop()
{
    // only stoppable if the node is started
    ROS_ASSERT(m_status_code == NodeStatusCode::STARTED,
               "cannot stop because status code is not STARTED");

    // terminate step thread
    m_impl->step_running = false;
    if (m_impl->step_thread) {
        m_impl->step_thread->join();
        m_impl->step_thread = nullptr;
    }
    // if (m_impl->step_timer != nullptr) {
    //     m_impl->step_timer->cancel();
    //     m_impl->step_timer = nullptr;
    // }

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STOPPED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

bool MasterNode::_ping(std::shared_ptr<Downstream> ds)
{
    auto goal_msg = ACT_AcceptDocument::Goal();
    goal_msg.x_control.code = 1; // ping
    goal_msg.x_control.text_msg = "ping";

    // opt.goal_response_callback = callback;
    auto res = ds->handler->async_send_goal(goal_msg, ds->options);

    auto t = (long)m_runtime_config->timeout_ms_send_to_downstream;
    auto wait_result = res.wait_for(std::chrono::milliseconds(t));
    if (wait_result == std::future_status::ready) {
        auto s = res.get()->get_status();
        bool ok = false;

        // downstream accepted?
        ok |= s == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
        ok |= s == rclcpp_action::GoalStatus::STATUS_SUCCEEDED;
        ok |= s == rclcpp_action::GoalStatus::STATUS_EXECUTING;

        if (!ok)
            return false;
    } else
        return false;
    return true;
}

void MasterNode::_step()
{
    // RCLCPP_INFO(m_impl->logger, "MasterNode::_step");
    std::vector<Map_Document_Waiting::key_type> tasks_to_remove;
    std::vector<decltype(m_psgdoc_task_waiting)::value_type> psgdoc_task_waiting_;
    {
        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();

        for (auto const &it : (**lock_ptr_document_task_waiting)) {
            psgdoc_task_waiting_.push_back(it);
        }
    }

    for (auto &it : psgdoc_task_waiting_) {
        auto &task = it.second;
        auto ds = task->downstream;

        while (true) {
            if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) // not retry need to ping
                if (!_ping(ds))
                    continue;

            ACT_AcceptDocument::Goal goal;
            goal.document.frame = task->frame;
            goal.document.signal_code = task->frame.signal_code;
            auto now = this->now();
            goal.document.header.stamp = now;
            // time log
            RCLCPP_INFO(m_impl->logger, "---TIME LOG: framenum %ld node %s type %s time %ld", goal.document.frame.frame_num, "master_node", "OUT", now.nanoseconds());

            auto handle = task->downstream->handler->async_send_goal(goal, ds->options);

            // add timeout condition
            auto t = (long)m_runtime_config->timeout_ms_send_to_downstream;
            // RCLCPP_INFO(m_impl->logger, "send goal %ld", task->frame.frame_num);
            auto wait_result = handle.wait_for(std::chrono::milliseconds(t));
            // RCLCPP_INFO(m_impl->logger, "after wait send goal %ld, wait_result is %d", task->frame.frame_num, wait_result);
            if (wait_result == std::future_status::ready) {
                auto task_response = handle.get();
                if (task_response != nullptr) {
                    // accepted or executing
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                        task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
                        // 这里状态为这两个不一定代表成功，可能是下游还在处理中，当下游返回aborted前也会是这两个状态
                        // 所以这里需要等待下游返回成功或者aborted
                        bool is_doc_task_done = false;
                        while (true) {
                            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                                // 如果发送成功了，is_doc_task_done为true，跳出发送document的循环
                                RCLCPP_INFO(m_impl->logger, "document %ld success because SUCCEED", task->frame.frame_num);
                                is_doc_task_done = true;
                                break;
                            } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING) {
                                // 如果发送失败了，is_doc_task_done为false，跳出发送document的循环，并让外面去判断是否需要重试发送document
                                RCLCPP_INFO(m_impl->logger, "document %ld failed because ABORTED", task->frame.frame_num);
                                is_doc_task_done = false;
                                break;
                            } else {
                                // 其他情况还需要等待状态变化
                                // sleep一些时间再去查询状态
                                // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));

                                // FIXME: 暂时当做发送成功处理
                                is_doc_task_done = true;
                                break;
                            }
                        }
                        if (is_doc_task_done) {
                            // 发送成功了，跳出发送doc的循环
                            m_psgdoc_task_done.push_back(task);
                            tasks_to_remove.push_back(it.first);
                            break;
                        } else {
                            // 发送失败了，判断是否需要重发
                            if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) {
                                // 不需要重发，跳出发送doc的循环
                                m_psgdoc_task_done.push_back(task);
                                tasks_to_remove.push_back(it.first);
                                RCLCPP_INFO(m_impl->logger, "document %ld drop because ABORTED", task->frame.frame_num);
                                break;
                            } else {
                                // 需要重发，继续发送
                                continue;
                            }
                        }
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        break;
                    }
                    // rejected
                    else {
                        if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) { // failed
                            m_psgdoc_task_done.push_back(task);
                            tasks_to_remove.push_back(it.first);
                            break;
                        } else { // retry
                            auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                            (**lock_ptr_document_task_waiting)[it.first]->retry_times++;
                            continue;
                        }
                    }
                } else {
                    // rejected
                    if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) { // failed
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        break;
                    } else { // retry
                        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                        (**lock_ptr_document_task_waiting)[it.first]->retry_times++;
                        continue;
                    }
                }
            } else {                                                                                    // timeout
                if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) { // failed
                    m_psgdoc_task_done.push_back(task);
                    tasks_to_remove.push_back(it.first);
                    break;
                } else { // retry
                    auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                    (**lock_ptr_document_task_waiting)[it.first]->retry_times++;
                    continue;
                }
            }
        }
    }

    // 出现在tasks_to_remove中的task都已经发送成功或被丢弃，从psgdoc_task_waiting中删除这些task
    {
        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_document_task_waiting)->erase(it);
            // RCLCPP_INFO(m_impl->logger, "erase waiting task %ld", std::get<1>(it));
        }
    }

    // 完成的task，从buffer中删除
    {
        auto lock_ptr_frame_buffer = m_impl->sync_frame_buffer.synchronize();
        // remove task done documents
        for (auto &it : m_psgdoc_task_done) {
            _remove_frame_from_buffer(it->frame.frame_num, *lock_ptr_frame_buffer, false);
        }
    }

    // for all done tasks, remove them from memory
    m_psgdoc_task_done.clear();
}
} // namespace FlowRos2Pipeline

#endif