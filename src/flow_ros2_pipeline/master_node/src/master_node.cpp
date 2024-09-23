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
    m_impl->sync_document_doing_map = &m_psgdoc_task_doing;
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
    this->declare_parameter<int>("send_goal_retry_times", 0);
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
    // RCLCPP_DEBUG(m_impl->logger, "Received status query request");
    response->status = ReturnCode::SUCCESS;
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
    const auto &control_msg = goal->control_msg;

    // if buffer is full, reject the frame
    if (m_frame_buffer.size() >= m_runtime_config->buffer_size) {
        auto result = std::make_shared<ACT_AcceptFrame::Result>();
        result->return_msg = "Buffer is full";
        result->return_code = ReturnCode::REJECTED;
        goal_handle->abort(result);
        return;
    }

    // ping
    if (control_msg.control_signal == 1) {
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
    // RCLCPP_INFO(m_impl->logger, "Accepted frame %ld and succeed", frame.frame_num);
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
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
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
        ACT_AcceptDocument::Goal goal;
        goal.document.frame = task->frame;
        goal.document.signal_code = task->frame.signal_code;
        auto now = this->now();
        goal.document.header.stamp = now;
        // time log
        RCLCPP_INFO(m_impl->logger, "---TIME LOG: framenum %ld node %s type %s time %ld", goal.document.frame.frame_num, "master_node", "OUT", now.nanoseconds());

        auto ds = task->downstream;
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
                    // successfully sent, record this
                    task->goal_handle = task_response;
                    {
                        auto lock_ptr_document_task_doing = m_impl->sync_document_doing_map.synchronize();
                        (**lock_ptr_document_task_doing)[task->goal_handle] = task;
                    }
                    tasks_to_remove.push_back(it.first);
                }

                // succeed
                else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                    m_psgdoc_task_done.push_back(task);
                    tasks_to_remove.push_back(it.first);
                }
                // rejected
                else {
                    auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                    (**lock_ptr_document_task_waiting)[it.first]->retry_times++;
                    if ((**lock_ptr_document_task_waiting)[it.first]->retry_times >= m_runtime_config->send_goal_retry_times) {
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                    }
                }
            } else {
                // rejected
                auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                (**lock_ptr_document_task_waiting)[it.first]->retry_times++;
                if ((**lock_ptr_document_task_waiting)[it.first]->retry_times >= m_runtime_config->send_goal_retry_times) {
                    m_psgdoc_task_done.push_back(task);
                    tasks_to_remove.push_back(it.first);
                }
            }
        } else {
            auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
            (**lock_ptr_document_task_waiting)[it.first]->retry_times++;
            if ((**lock_ptr_document_task_waiting)[it.first]->retry_times >= m_runtime_config->send_goal_retry_times) {
                m_psgdoc_task_done.push_back(task);
                tasks_to_remove.push_back(it.first);
            }
        }
    }

    // remove all sent tasks from waiting list
    {
        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_document_task_waiting)->erase(it);
            // RCLCPP_INFO(m_impl->logger, "erase waiting task %ld", std::get<1>(it));
        }
    }

    {
        auto lock_ptr_document_task_doing = m_impl->sync_document_doing_map.synchronize();
        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_document_task_doing)->empty()) {
            std::vector<GoalHandle> tasks_to_remove;
            for (auto &it : (**lock_ptr_document_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_psgdoc_task_done.push_back(it.second);
                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_document_task_doing)->erase(it);
            }
        }
    }

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