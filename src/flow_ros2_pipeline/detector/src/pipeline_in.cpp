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
    const auto &x_control = goal->x_control;

    // buffer size log
    RCLCPP_INFO(m_impl->logger, "frame %d m_psgdoc_task_waiting buffer size: %d", goal->document.frame.frame_num, m_psgdoc_task_waiting.size());
    RCLCPP_INFO(m_impl->logger, "frame %d m_frame_task_waiting buffer size: %d", goal->document.frame.frame_num, m_frame_task_waiting.size());

    // if buffer is full, reject the frame
    if (m_psgdoc_task_waiting.size() >= m_runtime_config->buffer_size || m_frame_task_waiting.size() >= m_runtime_config->buffer_size) {
        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Buffer is full";
        result->return_code = ReturnCode::REJECTED;
        goal_handle->abort(result);
        RCLCPP_INFO(m_impl->logger, "REJECTED!!! Buffer is full");
        return;
    }

    // ping
    if (x_control.code == 1) {
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

    // add x_uid to document
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    std::copy(uuid.begin(), uuid.end(), document.x_uid.uuid.begin());

    // create tasks for all downstreams
    {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        _process_document_create_tasks(document, *lock_ptr_psgdoc_task_waiting);
    }
    {
        auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
        _process_frame_create_tasks(frame, document.x_uid, *lock_ptr_frame_task_waiting);
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

void DetectorIn::_process_frame_create_tasks(const MSG_Frame &frame, const MSG_UUID &x_uid, Map_Frame_Waiting *frame_waiting_map_ptr)
{
    // RCLCPP_DEBUG(m_impl->logger, "create frame %ld detections uuid %s tasks for downstreams", frame.frame_num,
    //              uuid_to_string(x_uid.uuid).c_str());

    // create tasks of this frame for all downstreams
    for (auto &x : m_model_downstreams) {
        auto task = std::make_shared<DSTask_Frame>();
        task->downstream = x.second;
        task->frame = frame;

        task->x_uid = x_uid;
        (*frame_waiting_map_ptr)[std::make_tuple(task->downstream.get(), frame.frame_num)] = task;
    }
}

void DetectorIn::_process_create_tasks(const MSG_PsgDocument &document, const MSG_Frame &frame,
                                       const MSG_UUID &x_uid, Map_Task_Waiting *task_waiting_map_ptr)
{
    // create tasks of this frame for all downstreams
    for (auto &x : m_model_downstreams) {
        auto task = std::make_shared<DSTask_Frame>();
        task->downstream = x.second;
        task->frame = frame;

        task->x_uid = x_uid;
        (*task_waiting_map_ptr)[frame.frame_num] = std::make_pair(T1 && x, T2 && y) task;
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
    goal_msg.x_control.code = 1; // ping
    goal_msg.x_control.text_msg = "ping";

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
    goal_msg.x_control.code = 1; // ping
    goal_msg.x_control.text_msg = "ping";

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
                if (!_ping_model(ds)) {
                    RCLCPP_INFO(m_impl->logger, "ping failed");
                    continue;
                }

            ACT_AcceptFrame::Goal goal;
            goal.frame = task->frame;
            // add x_uid to goal
            goal.x_uid = task->x_uid;

            auto handle = task->downstream->accept_frame->async_send_goal(goal, ds->accept_frame_options);

            // RCLCPP_INFO(m_impl->logger, "[Request Send]framenum: %ld, x_uid: %s", goal.frame.frame_num,
            // uuid_to_string(goal.x_uid.uuid).c_str());

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
                        RCLCPP_INFO(m_impl->logger, "frame %ld success because ACCEPTED OR EXECUTING", task->frame.frame_num);
                        if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
                            RCLCPP_INFO(m_impl->logger, "task_response is STATUS_EXECUTING");
                        else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED)
                            RCLCPP_INFO(m_impl->logger, "task_response is STATUS_ACCEPTED");
                        break;
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_frame_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        RCLCPP_INFO(m_impl->logger, "frame %ld success because SUCCEEDED", task->frame.frame_num);
                        break;
                    }
                    // rejected
                    else {
                        if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) { // failed
                            m_frame_task_done.push_back(task);
                            tasks_to_remove.push_back(it.first);
                            RCLCPP_INFO(m_impl->logger, "frame %ld drop because REJECTED", task->frame.frame_num);
                            break;
                        } else { // retry
                            auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
                            (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                            RCLCPP_INFO(m_impl->logger, "frame %ld retry because REJECTED", task->frame.frame_num);
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
                        RCLCPP_INFO(m_impl->logger, "frame %ld retry because REJECTED", task->frame.frame_num);
                        continue;
                    }
                }
            } else {
                // timeout
                if (!m_runtime_config->send_goal_retry && task->frame.signal_code == SignalCode::RUN) { // failed
                    m_frame_task_done.push_back(task);
                    tasks_to_remove.push_back(it.first);
                    RCLCPP_INFO(m_impl->logger, "frame %ld drop because TIMEOUT", task->frame.frame_num);
                    break;
                } else { // retry
                    auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
                    (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                    RCLCPP_INFO(m_impl->logger, "frame %ld retry because TIMEOUT", task->frame.frame_num);
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
                    // RCLCPP_INFO(m_impl->logger, "frame %ld doing task_response is %d", it.second->frame.frame_num, task_response->get_status());
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_frame_task_done.push_back(it.second);
                        tasks_to_remove.push_back(it.first);
                    } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED) {
                        if (!m_runtime_config->send_goal_retry && it.second->frame.signal_code == SignalCode::RUN) { // failed
                            m_frame_task_done.push_back(it.second);
                            tasks_to_remove.push_back(it.first);
                        } else {
                            auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
                            (**lock_ptr_frame_task_waiting)[std::make_tuple(it.second->downstream.get(),
                                                                            it.second->frame.frame_num)] = it.second;

                            tasks_to_remove.push_back(it.first);
                        }
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

// 单次只发一个任务到下游
void DetectorIn::_send_single_task_to_downstream()
{
    // 取第一个等待的任务
    decltype(m_psgdoc_task_waiting)::value_type first_item;
    {
        // TODO: 这里需要加一个新的队列，格式为<doc, frame>的pair
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        if ((*lock_ptr_psgdoc_task_waiting)->empty())
            return false;
        first_item = *((*lock_ptr_psgdoc_task_waiting)->begin());
    }

    auto &doc_task = first_item.second.first;
    auto &frame_task = first_item.second.second;
    auto doc_ds = doc_task->downstream;
    auto frame_ds = doc_task->downstream;

    bool is_doc_task_done = false;
    // 先发送document，如果成功再发送frame，否则不发送frame
    while (true) {
        if (!m_runtime_config->send_goal_retry && doc_task->document.frame.signal_code == SignalCode::RUN) // not retry need to ping
            if (!_ping_pipeline(doc_ds))
                continue;

        ACT_AcceptDocument::Goal goal;
        goal.document = doc_task->document;
        auto handle = doc_task->downstream->accept_document->async_send_goal(goal, doc_ds->accept_document_options);
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
                    // 这里状态为这两个不一定代表成功，可能是下游还在处理中，当下游返回aborted前也会是这两个状态
                    // 所以这里需要等待下游返回成功或者aborted
                    while (true) {
                        if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                            // 如果发送成功了，is_doc_task_done为true，跳出发送document的循环
                            RCLCPP_INFO(m_impl->logger, "document %ld success because SUCCEED", doc_task->document.frame.frame_num);
                            is_doc_task_done = true;
                            break;
                        } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                                   task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELED ||
                                   task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING) {
                            // 如果发送失败了，is_doc_task_done为false，跳出发送document的循环，并让外面去判断是否需要重试发送document
                            RCLCPP_INFO(m_impl->logger, "document %ld failed because ABORTED", doc_task->document.frame.frame_num);
                            is_doc_task_done = false;
                            break;
                        } else {
                            // 其他情况还需要等待状态变化
                            // sleep一些时间再去查询状态
                            // std::this_thread::sleep_for(m_runtime_config->step_interval_ms);

                            // FIXME: 暂时当做发送成功处理
                            is_doc_task_done = true;
                            break;
                        }
                    }
                } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                    // 如果发送成功了，is_doc_task_done为true，跳出发送document的循环
                    RCLCPP_INFO(m_impl->logger, "document %ld success because SUCCEED", doc_task->document.frame.frame_num);
                    is_doc_task_done = true;
                    break;
                } else {
                    // if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                    //        task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELED ||
                    //        task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING)
                    // 如果发送失败了，is_doc_task_done为false，跳出发送document的循环，并让外面去判断是否需要重试发送document
                    RCLCPP_INFO(m_impl->logger, "document %ld failed because ABORTED", doc_task->document.frame.frame_num);
                    is_doc_task_done = false;
                    break;
                }
            } else {
                // 如果发送失败了，is_doc_task_done为false，跳出发送document的循环，并让外面去判断是否需要重试发送document
                RCLCPP_INFO(m_impl->logger, "document %ld failed because ABORTED", doc_task->document.frame.frame_num);
                is_doc_task_done = false;
                break;
            }
        } else {
            // 如果发送超时了，is_doc_task_done为false，跳出发送document的循环，并让外面去判断是否需要重试发送document
            RCLCPP_INFO(m_impl->logger, "document %ld failed because TIMEOUT", doc_task->document.frame.frame_num);
            is_doc_task_done = false;
            break;
        }
    }

    // 判断是否发送成功了document
    if (is_doc_task_done) {

        // 发送成功了document，再发送frame
        while (true) {
            if (!m_runtime_config->send_goal_retry && frame_task->frame.signal_code == SignalCode::RUN) // not retry need to ping
                if (!_ping_model(frame_ds)) {
                    RCLCPP_INFO(m_impl->logger, "ping failed");
                    continue;
                }

            ACT_AcceptFrame::Goal frame_goal;
            frame_goal.frame = frame_task->frame;
            // add x_uid to goal
            frame_goal.x_uid = frame_task->x_uid;

            auto frame_handle = frame_task->downstream->accept_frame->async_send_goal(frame_goal, frame_ds->accept_frame_options);

            // RCLCPP_INFO(m_impl->logger, "[Request Send]framenum: %ld, x_uid: %s", goal.frame.frame_num,
            // uuid_to_string(goal.x_uid.uuid).c_str());

            // add timeout condition
            auto t = (long)m_runtime_config->timeout_ms_send_to_downstream;
            // RCLCPP_INFO(m_impl->logger, "send goal %ld", task->frame.frame_num);
            auto wait_result = frame_handle.wait_for(std::chrono::milliseconds(t));
            // RCLCPP_INFO(m_impl->logger, "after wait send goal %ld, wait_result is %d", task->frame.frame_num, wait_result);
            if (wait_result == std::future_status::ready) {
                auto task_response = frame_handle.get();
                if (task_response != nullptr) {
                    // accepted or executing
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                        task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
                        // 这里状态为这两个不一定代表成功，可能是下游还在处理中，当下游返回aborted前也会是这两个状态
                        // 所以这里需要等待下游返回成功或者aborted

                        bool is_frame_task_done = false;
                        while (true) {
                            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                                // 如果发送成功了，is_frame_task_done为true，跳出发送frame的循环
                                RCLCPP_INFO(m_impl->logger, "frame %ld success because SUCCEED", frame_task->frame.frame_num);
                                is_frame_task_done = true;
                                break;
                            } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING) {
                                // 如果发送失败了，is_frame_task_done为false，跳出发送frame的循环，并让外面去判断是否需要重试发送frame
                                RCLCPP_INFO(m_impl->logger, "frame %ld failed because ABORTED", frame_task->frame.frame_num);
                                is_frame_task_done = false;
                                break;
                            } else {
                                // 其他情况还需要等待状态变化
                                // sleep一些时间再去查询状态
                                // std::this_thread::sleep_for(m_runtime_config->step_interval_ms);

                                // FIXME: 暂时当做发送成功处理
                                is_doc_task_done = true;
                                break;
                            }
                        }
                        if (is_frame_task_done) // 如果发送成功了，跳出发送frame的循环
                            break;
                        else // 如果发送失败了，需要重复发送frame直到成功
                            continue;
                    } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        // 如果发送成功了，is_frame_task_done为true，跳出发送frame的循环
                        RCLCPP_INFO(m_impl->logger, "frame %ld success because SUCCEED", frame_task->frame.frame_num);
                        break;
                    } else {
                        // if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                        //        task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELED ||
                        //        task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING)
                        // 如果发送失败了，需要重复发送frame直到成功
                        RCLCPP_INFO(m_impl->logger, "frame %ld failed because ABORTED", frame_task->frame.frame_num);
                        continue;
                    }
                } else {
                    // 如果发送失败了，需要重复发送frame直到成功
                    RCLCPP_INFO(m_impl->logger, "frame %ld failed because ABORTED", frame_task->frame.frame_num);
                    continue;
                }
            } else {
                // 如果发送超时了，需要重复发送frame直到成功
                RCLCPP_INFO(m_impl->logger, "frame %ld failed because TIMEOUT", frame_task->frame.frame_num);
                continue;
            }
        }

        // 跳出frame的循环则说明frame发送成功了，删除这个任务
        {
            // TODO: 这里需要加一个新的队列，格式为<doc, frame>的pair
            // auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
            // (*lock_ptr_psgdoc_task_waiting)->erase(first_item.first);
        }
    }
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
                        RCLCPP_INFO(m_impl->logger, "document %ld success because ACCEPTED OR EXECUTING", task->document.frame.frame_num);
                        if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING)
                            RCLCPP_INFO(m_impl->logger, "task_response is STATUS_EXECUTING");
                        else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED)
                            RCLCPP_INFO(m_impl->logger, "task_response is STATUS_ACCEPTED");
                        break;
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        RCLCPP_INFO(m_impl->logger, "document %ld success because SUCCEED", task->document.frame.frame_num);
                        break;
                    }
                    // rejected
                    else {
                        if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                            m_psgdoc_task_done.push_back(task);
                            tasks_to_remove.push_back(it.first);
                            RCLCPP_INFO(m_impl->logger, "document %ld drop because REJECTED", task->document.frame.frame_num);
                            break;
                        } else { // retry
                            auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                            (**lock_ptr_psgdoc_task_waiting)[it.first]->retry_times++;
                            RCLCPP_INFO(m_impl->logger, "document %ld retry because REJECTED", task->document.frame.frame_num);
                            continue;
                        }
                    }
                } else {
                    // rejected
                    if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        RCLCPP_INFO(m_impl->logger, "document %ld drop because REJECTED", task->document.frame.frame_num);
                        break;
                    } else { // retry
                        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                        (**lock_ptr_psgdoc_task_waiting)[it.first]->retry_times++;
                        RCLCPP_INFO(m_impl->logger, "document %ld retry because REJECTED", task->document.frame.frame_num);
                        continue;
                    }
                }
            } else {                                                                                             // timeout
                if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                    m_psgdoc_task_done.push_back(task);
                    tasks_to_remove.push_back(it.first);
                    RCLCPP_INFO(m_impl->logger, "document %ld drop because TIMEOUT", task->document.frame.frame_num);
                    break;
                } else { // retry
                    auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                    (**lock_ptr_psgdoc_task_waiting)[it.first]->retry_times++;
                    RCLCPP_INFO(m_impl->logger, "document %ld retry because TIMEOUT", task->document.frame.frame_num);
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
                    } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED) {
                        RCLCPP_INFO(m_impl->logger, "document %ld doing task_response is STATUS_ABORTED", it.second->document.frame.frame_num);
                        if (!m_runtime_config->send_goal_retry && it.second->document.frame.signal_code == SignalCode::RUN) { // failed
                            m_psgdoc_task_done.push_back(it.second);
                            tasks_to_remove.push_back(it.first);
                        } else { // retry
                            auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                            (**lock_ptr_psgdoc_task_waiting)[std::make_tuple(it.second->downstream.get(),
                                                                             it.second->document.frame.frame_num)] = it.second;

                            tasks_to_remove.push_back(it.first);
                        }
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