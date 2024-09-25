#include <boost/thread/synchronized_value.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <rclcpp/utilities.hpp>

#include <detector/_pipeline_in.hpp>
#include <detector/pipeline_in.hpp>
#include <rcpputils/asserts.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
DetectorIn::DetectorIn()
    : Node("detector_in_node")
{
    m_impl = std::make_shared<DetectorInImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    m_impl->sync_document_doing_map = &m_psgdoc_task_doing;
    m_impl->sync_frame_waiting_map = &m_frame_task_waiting;
    m_impl->sync_frame_doing_map = &m_frame_task_doing;

    // RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int DetectorIn::init(const std::shared_ptr<InitConfig> &config,
                     const std::shared_ptr<RuntimeConfig> &runtime_config)
{
    ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::STOPPED,
               "init FAILED! status code is not BEFORE_INIT or STOPPED");

    m_init_config = config;
    m_runtime_config = runtime_config;

    // setup downstreams
    _connect_to_downstreams();

    // create server
    m_act_process_document = rclcpp_action::create_server<ACT_AcceptDocument>(
        this, m_init_config->process_document_action,
        std::bind(&DetectorIn::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DetectorIn::_accept_document_cancel_callback, this, std::placeholders::_1),
        std::bind(&DetectorIn::_accept_document_accepted_callback, this, std::placeholders::_1));

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<DetectorIn::InitConfig> &DetectorIn::get_init_config() const
{
    return m_init_config;
}

int DetectorIn::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<DetectorIn::RuntimeConfig> &DetectorIn::get_runtime_config() const
{
    return m_runtime_config;
}


int DetectorIn::start()
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

    return ReturnCode::SUCCESS;
}

int DetectorIn::stop()
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

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STOPPED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}


int DetectorIn::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse DetectorIn::_accept_document_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDocument::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DetectorIn::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void DetectorIn::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    const auto &goal = goal_handle->get_goal();
    const auto &control_msg = goal->control_msg;

    // // if buffer is full, reject the frame
    // if (m_psgdoc_task_waiting.size() >= m_runtime_config->buffer_size || m_frame_task_waiting.size() >= m_runtime_config->buffer_size) {
    //     auto result = std::make_shared<ACT_AcceptDocument::Result>();
    //     result->return_msg = "Buffer is full";
    //     result->return_code = ReturnCode::REJECTED;
    //     goal_handle->abort(result);
    //     return;
    // }

    // ping
    if (control_msg.control_signal == 1) {
        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Ping accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
        return;
    }

    // time log
    RCLCPP_INFO(m_impl->logger, "---TIME LOG: framenum %ld node %s type %s time %ld", goal->document.frame.frame_num, "detector", "IN", this->now().nanoseconds());

    // cache the document, copy it for modify it
    auto document = goal->document;
    const auto &frame = document.frame;

    // add detections_uuid to document
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    std::copy(uuid.begin(), uuid.end(), document.detections_uuid.uuid.begin());

    // create tasks for all downstreams
    {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        _process_document_create_tasks(document, *lock_ptr_psgdoc_task_waiting);
    }
    {
        auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
        _process_frame_create_tasks(frame, document.detections_uuid, *lock_ptr_frame_task_waiting);
    }

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}


void DetectorIn::_process_document_create_tasks(const MSG_PsgDocument &document, Map_Document_Waiting *psgdoc_task_waiting_ptr)
{
    // create tasks of this frame for all downstreams
    for (auto &x : m_pipeline_downstreams) {

        auto task = std::make_shared<DSTask_PsgDocument>();
        task->downstream = x.second;
        task->document = document;
        (*psgdoc_task_waiting_ptr)[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
    }
}

void DetectorIn::_process_frame_create_tasks(const MSG_Frame &frame, const MSG_UUID &detections_uuid, Map_Frame_Waiting *frame_waiting_map_ptr)
{
    // RCLCPP_DEBUG(m_impl->logger, "create frame %ld detections uuid %s tasks for downstreams", frame.frame_num,
    //              uuid_to_string(detections_uuid.uuid).c_str());

    // create tasks of this frame for all downstreams
    for (auto &x : m_model_downstreams) {
        auto task = std::make_shared<DSTask_Frame>();
        task->downstream = x.second;
        task->frame = frame;

        task->detections_uuid = detections_uuid;
        (*frame_waiting_map_ptr)[std::make_tuple(task->downstream.get(), frame.frame_num)] = task;
    }
}


void DetectorIn::_step()
{
    _send_frame_to_downstreams();
    _send_document_to_downstreams();
}

void DetectorIn::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

    m_pipeline_downstreams.clear();
    m_model_downstreams.clear();

    for (auto it : m_init_config->pipeline_downstreams) {
        auto ds = std::make_shared<DownstreamPipeline>();
        // RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

        // 创建pipeline downstream
        {
            std::string name = it.second.accept_document_action;
            auto client = rclcpp_action::create_client<ACT_AcceptDocument>(this, name);

            ds->accept_document = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&DetectorIn::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&DetectorIn::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&DetectorIn::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            // RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            // RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_pipeline_downstreams[it.first] = ds;
    }

    for (auto it : m_init_config->model_downstreams) {
        auto ds = std::make_shared<DownstreamModel>();
        // RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

        // 创建pipeline downstream
        {
            std::string name = it.second.accept_frame_action;
            auto client = rclcpp_action::create_client<ACT_AcceptFrame>(this, name);

            ds->accept_frame = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&DetectorIn::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&DetectorIn::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&DetectorIn::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            // RCLCPP_INFO(m_impl->logger, "waiting for model action server %s", name.c_str());
            client->wait_for_action_server();
            // RCLCPP_INFO(m_impl->logger, "model action server %s is ready", name.c_str());
        }

        m_model_downstreams[it.first] = ds;
    }
}

bool DetectorIn::_ping_model(std::shared_ptr<DownstreamModel> ds)
{
    auto goal_msg = ACT_AcceptFrame::Goal();
    goal_msg.control_msg.control_signal = 1; // ping
    goal_msg.control_msg.control_msg = "ping";

    // opt.goal_response_callback = callback;
    auto res = ds->accept_frame->async_send_goal(goal_msg, ds->accept_frame_options);

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

bool DetectorIn::_ping_pipeline(std::shared_ptr<DownstreamPipeline> ds)
{
    auto goal_msg = ACT_AcceptDocument::Goal();
    goal_msg.control_msg.control_signal = 1; // ping
    goal_msg.control_msg.control_msg = "ping";

    // opt.goal_response_callback = callback;
    auto res = ds->accept_document->async_send_goal(goal_msg, ds->accept_document_options);

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

void DetectorIn::_send_frame_to_downstreams()
{
    std::vector<decltype(m_frame_task_waiting)::value_type> frame_task_waiting_;
    {
        auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();

        for (auto const &it : (**lock_ptr_frame_task_waiting)) {
            frame_task_waiting_.push_back(it);
        }
    }

    std::vector<Map_Frame_Waiting::key_type> tasks_to_remove;

    for (auto &it : frame_task_waiting_) {
        auto &task = it.second;
        auto ds = task->downstream;

        while (true) {
            if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) // not retry need to ping
                if (!_ping_model(ds))
                    continue;

            ACT_AcceptFrame::Goal goal;
            goal.frame = task->frame;
            // add detections_uuid to goal
            goal.detections_uuid = task->detections_uuid;

            auto handle = task->downstream->accept_frame->async_send_goal(goal, ds->accept_frame_options);

            // RCLCPP_INFO(m_impl->logger, "[Request Send]framenum: %ld, detections_uuid: %s", goal.frame.frame_num,
            // uuid_to_string(goal.detections_uuid.uuid).c_str());

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
                            auto lock_ptr_frame_task_doing = m_impl->sync_frame_doing_map.synchronize();
                            (**lock_ptr_frame_task_doing)[task->goal_handle] = task;
                        }
                        tasks_to_remove.push_back(it.first);
                        break;
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_frame_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        break;
                    }
                    // rejected
                    else {
                        if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) { // failed
                            m_frame_task_done.push_back(task);
                            tasks_to_remove.push_back(it.first);
                            break;
                        } else { // retry
                            auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
                            (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                            continue;
                        }
                    }
                } else {
                    // rejected
                    if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) { // failed
                        m_frame_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        break;
                    } else { // retry
                        auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
                        (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                        continue;
                    }
                }
            } else {
                // timeout
                if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) { // failed
                    m_frame_task_done.push_back(task);
                    tasks_to_remove.push_back(it.first);
                    break;
                } else { // retry
                    auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
                    (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                    continue;
                }
            }
        }
    }

    // remove all sent tasks
    {
        auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_frame_task_waiting)->erase(it);
        }
    }

    {
        auto lock_ptr_frame_task_doing = m_impl->sync_frame_doing_map.synchronize();
        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_frame_task_doing)->empty()) {
            std::vector<GoalHandle_Frame> tasks_to_remove;
            for (auto &it : (**lock_ptr_frame_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_frame_task_done.push_back(it.second);
                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_frame_task_doing)->erase(it);
            }
        }
    }

    // for all done tasks, remove them from memory
    m_frame_task_done.clear();
}

void DetectorIn::_send_document_to_downstreams()
{
    // initiate all waiting tasks
    std::vector<Map_Document_Waiting::key_type> tasks_to_remove;

    std::vector<decltype(m_psgdoc_task_waiting)::value_type> psgdoc_task_waiting_;
    {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();

        for (auto const &it : (**lock_ptr_psgdoc_task_waiting)) {
            psgdoc_task_waiting_.push_back(it);
        }
    }

    for (auto &it : psgdoc_task_waiting_) {
        auto &task = it.second;
        auto ds = task->downstream;

        while (true) {
            if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) // not retry need to ping
                if (!_ping_pipeline(ds))
                    continue;

            ACT_AcceptDocument::Goal goal;
            goal.document = task->document;
            auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);
            // RCLCPP_INFO(m_impl->logger, "_step document async_send_goal: %ld", task->document.frame.frame_num);

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
                            auto lock_ptr_psgdoc_task_doing = m_impl->sync_document_doing_map.synchronize();
                            (**lock_ptr_psgdoc_task_doing)[task->goal_handle] = task;
                        }
                        tasks_to_remove.push_back(it.first);
                        break;
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        break;
                    }
                    // rejected
                    else {
                        if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                            m_psgdoc_task_done.push_back(task);
                            tasks_to_remove.push_back(it.first);
                            break;
                        } else { // retry
                            auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                            (**lock_ptr_psgdoc_task_waiting)[it.first]->retry_times++;
                            continue;
                        }
                    }
                } else {
                    // rejected
                    if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        break;
                    } else { // retry
                        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                        (**lock_ptr_psgdoc_task_waiting)[it.first]->retry_times++;
                        continue;
                    }
                }
            } else {                                                                                             // timeout
                if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                    m_psgdoc_task_done.push_back(task);
                    tasks_to_remove.push_back(it.first);
                    break;
                } else { // retry
                    auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                    (**lock_ptr_psgdoc_task_waiting)[it.first]->retry_times++;
                    continue;
                }
            }
        }
    }

    // remove all sent tasks
    {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_psgdoc_task_waiting)->erase(it);
        }
    }

    {
        auto lock_ptr_psgdoc_task_doing = m_impl->sync_document_doing_map.synchronize();
        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_psgdoc_task_doing)->empty()) {
            std::vector<GoalHandle_PsgDocument> tasks_to_remove;
            for (auto &it : (**lock_ptr_psgdoc_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_psgdoc_task_done.push_back(it.second);
                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_psgdoc_task_doing)->erase(it);
            }
        }
    }

    // for all done tasks, remove them from memory
    m_psgdoc_task_done.clear();
}


void DetectorIn::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", 10000);
    this->declare_parameter<int>("buffer_size", 1);
    this->declare_parameter<bool>("send_goal_retry", false);
}
} // namespace FlowRos2Pipeline