#include "psg_common/psg_common.hpp"
#include <boost/thread/lock_algorithms.hpp>

#include <detector/_pipeline_out.hpp>
#include <detector/pipeline_out.hpp>
#include <rcpputils/asserts.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
DetectorOut::DetectorOut()
    : Node("detector_out_node")
{
    m_impl = std::make_shared<DetectorOutImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    m_impl->sync_document_doing_map = &m_psgdoc_task_doing;

    m_impl->sync_document_buffer = &m_document_buffer;
    m_impl->sync_detections_buffer = &m_detections_buffer;


    // RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int DetectorOut::init(const std::shared_ptr<InitConfig> &config,
                      const std::shared_ptr<RuntimeConfig> &runtime_config)
{
    if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::STOPPED) {
        RCLCPP_ERROR(m_impl->logger, "init FAILED! status code is not BEFORE_INIT or STOPPED");
        return ReturnCode::ERROR;
    }
    ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT,
               "init FAILED! status code is not BEFORE_INIT");

    m_init_config = config;
    m_runtime_config = runtime_config;

    // create process document server
    m_act_process_document = rclcpp_action::create_server<ACT_AcceptDocument>(
        this, m_init_config->process_document_action,
        std::bind(&DetectorOut::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DetectorOut::_accept_document_cancel_callback, this, std::placeholders::_1),
        std::bind(&DetectorOut::_accept_document_accepted_callback, this, std::placeholders::_1));

    // create process detections server
    // std::string process_detections_action = this->get_parameter(m_init_config->process_detections_action).as_string();
    m_act_process_detections = rclcpp_action::create_server<ACT_AcceptDetections>(
        this, m_init_config->process_detections_action,
        std::bind(&DetectorOut::_accept_detections_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DetectorOut::_accept_detections_cancel_callback, this, std::placeholders::_1),
        std::bind(&DetectorOut::_accept_detections_accepted_callback, this, std::placeholders::_1));

    // setup downstreams
    _connect_to_downstreams();

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<DetectorOut::InitConfig> &DetectorOut::get_init_config() const
{
    return m_init_config;
}

int DetectorOut::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<DetectorOut::RuntimeConfig> &DetectorOut::get_runtime_config() const
{
    return m_runtime_config;
}


int DetectorOut::start()
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

int DetectorOut::stop()
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


int DetectorOut::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse DetectorOut::_accept_document_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDocument::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DetectorOut::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void DetectorOut::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();
    const auto &control_msg = goal->control_msg;

    // buffer size log
    RCLCPP_INFO(m_impl->logger, "frame %d m_document_buffer buffer size: %d", goal->document.frame.frame_num, m_document_buffer.size());

    // if buffer is full, reject the frame
    if (m_document_buffer.size() >= m_runtime_config->buffer_size) {
        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Buffer is full";
        result->return_code = ReturnCode::REJECTED;
        goal_handle->abort(result);
        RCLCPP_INFO(m_impl->logger, "REJECTED!!! Buffer is full");
        return;
    }

    // ping
    if (control_msg.control_signal == 1) {
        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Ping accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
        return;
    }

    // cache the document
    auto document = goal->document;

    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        _add_document_to_buffer(document, *lock_ptr_document_buffer);

        // RCLCPP_DEBUG(m_impl->logger, "Accepted document %ld with UUID %s and add it to buffer",
        //              document.frame.frame_num, uuid_to_string(document.detections_uuid.uuid).c_str());
    }

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}

rclcpp_action::GoalResponse DetectorOut::_accept_detections_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDetections::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with detections with frame_num %ld", goal->detections.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DetectorOut::_accept_detections_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void DetectorOut::_accept_detections_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();
    const auto &control_msg = goal->control_msg;

    // // if buffer is full, reject the frame
    // if (m_detections_buffer.size() >= m_runtime_config->buffer_size) {
    //     auto result = std::make_shared<ACT_AcceptDetections::Result>();
    //     result->return_msg = "Buffer is full";
    //     result->return_code = ReturnCode::REJECTED;
    //     goal_handle->abort(result);
    //     return;
    // }

    // ping
    if (control_msg.control_signal == 1) {
        auto result = std::make_shared<ACT_AcceptDetections::Result>();
        result->return_msg = "Ping accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
        return;
    }

    // cache the detections
    const auto &detections = goal->detections;

    // add to buffer
    {
        auto lock_ptr_detections_buffer = m_impl->sync_detections_buffer.synchronize();
        _add_detections_to_buffer(detections, *lock_ptr_detections_buffer);
    }

    // RCLCPP_INFO(m_impl->logger, "add detections to buffer");
    // RCLCPP_INFO(m_impl->logger, "Accepted detections %ld with UUID %s and add it to buffer",
    // detections.frame.frame_num, uuid_to_string(detections.uuid.uuid).c_str());

    auto result = std::make_shared<ACT_AcceptDetections::Result>();
    result->return_msg = "Detections accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}


void DetectorOut::_process_document_create_tasks(const MSG_PsgDocument &document,
                                                 DetectorOut::Map_Document_Waiting *document_waiting_map_ptr)
{
    // RCLCPP_DEBUG(m_impl->logger, "create tasks for document %ld", document.frame.frame_num);

    // create tasks of this frame for all downstreams
    for (auto &x : m_downstreams) {
        auto task = std::make_shared<DSTask_PsgDocument>();
        task->downstream = x.second;
        task->document = document;
        (*document_waiting_map_ptr)[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
    }
}


void DetectorOut::_step()
{
    _merge_detections_and_documents();
    _send_document_to_downstreams();
}

void DetectorOut::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

    m_downstreams.clear();

    for (auto it : m_init_config->downstreams) {
        auto ds = std::make_shared<Downstream>();
        // RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

        // 创建downstream
        {
            std::string name = it.second.accept_document_action;
            auto client = rclcpp_action::create_client<ACT_AcceptDocument>(this, name);

            ds->accept_document = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&DetectorOut::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&DetectorOut::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&DetectorOut::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            // RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            // RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_downstreams[it.first] = ds;
    }
}

bool DetectorOut::_ping(const std::shared_ptr<Downstream> &ds)
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

void DetectorOut::_send_document_to_downstreams()
{
    std::vector<Map_Document_Waiting::key_type> tasks_to_remove;
    std::vector<decltype(m_psgdoc_task_waiting)::value_type> psgdoc_task_waiting_;
    {
        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();

        for (auto const &it : (**lock_ptr_document_task_waiting)) {
            psgdoc_task_waiting_.push_back(it);
            // RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): psgdoc_task_waiting_ push_back framenumber %d", std::get<1>(it.first));
        }
    }

    for (auto &it : psgdoc_task_waiting_) {
        // RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): psgdoc_task_waiting_ framenumber %d", std::get<1>(it.first));
        auto &task = it.second;
        ACT_AcceptDocument::Goal goal;
        goal.document = task->document;
        auto ds = task->downstream;

        while (true) {
            if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) // not retry need to ping
                if (!_ping(ds))
                    continue;

            // time log
            RCLCPP_INFO(m_impl->logger, "---TIME LOG: framenum %ld node %s type %s time %ld", goal.document.frame.frame_num, "detector", "OUT", this->now().nanoseconds());

            auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);

            // add timeout condition
            auto t = (long)m_runtime_config->timeout_ms_send_to_downstream;
            // RCLCPP_INFO(m_impl->logger, "send goal %ld", task->frame.frame_num);
            auto wait_result = handle.wait_for(std::chrono::milliseconds(t));
            // RCLCPP_INFO(m_impl->logger, "after wait send goal %ld, wait_result is %d", task->frame.frame_num, wait_result);
            if (wait_result == std::future_status::ready) {
                auto task_response = handle.get();
                if (task_response != nullptr) {
                    // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): document async_send_goal: %ld task_response is %d",
                    //              task->document.frame.frame_num, task_response->get_status());
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
                        // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): STATUS_ACCEPTED tasks_to_remove push_back framenumber %d", std::get<1>(it.first));
                        break;
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        // task->status = DSTask_PSGDocument::TASK_DONE;
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): STATUS_SUCCEEDED tasks_to_remove push_back framenumber %d", std::get<1>(it.first));
                        break;
                    }
                    // rejected
                    else {
                        if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
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
                    if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        break;
                    } else { // retry
                        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                        (**lock_ptr_document_task_waiting)[it.first]->retry_times++;
                        continue;
                    }
                }
            } else {                                                                                             // timeout
                if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
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

    // remove all sent tasks
    {
        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): tasks_to_remove framenumber %d", std::get<1>(it));
            (*lock_ptr_document_task_waiting)->erase(it);
        }
    }

    {
        auto lock_ptr_document_task_doing = m_impl->sync_document_doing_map.synchronize();
        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_document_task_doing)->empty()) {
            std::vector<GoalHandle_PsgDocument> tasks_to_remove;
            for (auto &it : (**lock_ptr_document_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_psgdoc_task_done.push_back(it.second);
                        tasks_to_remove.push_back(it.first);
                    } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED) {
                        if (!m_runtime_config->send_goal_retry && it.second->document.frame.signal_code == SignalCode::RUN) { // failed
                            m_psgdoc_task_done.push_back(it.second);
                            tasks_to_remove.push_back(it.first);
                        } else {
                            auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                            (**lock_ptr_psgdoc_task_waiting)[std::make_tuple(it.second->downstream.get(),
                                                                             it.second->document.frame.frame_num)] = it.second;

                            tasks_to_remove.push_back(it.first);
                        }
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_document_task_doing)->erase(it);
            }
        }
    }

    // for all done tasks, remove them from memory
    m_psgdoc_task_done.clear();
}


void DetectorOut::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<std::string>("process_detections_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
    this->declare_parameter<int>("buffer_size", 1);
    this->declare_parameter<bool>("send_goal_retry", false);
}


void DetectorOut::_add_document_to_buffer(const MSG_PsgDocument &document, std::map<int, DetectorOut::MSG_PsgDocument> *document_buffer_ptr)
{
    (*document_buffer_ptr)[document.frame.frame_num] = document;
}

void DetectorOut::_add_detections_to_buffer(const MSG_Detections &detections, std::map<int, DetectorOut::MSG_Detections> *detections_buffer_ptr)
{
    (*detections_buffer_ptr)[detections.frame.frame_num] = detections;
}

void DetectorOut::_remove_document_from_buffer(int frame_number, std::map<int, DetectorOut::MSG_PsgDocument> *document_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (document_buffer_ptr->find(frame_number) != document_buffer_ptr->end()) {
        document_buffer_ptr->erase(frame_number);
    }
}

void DetectorOut::_remove_detections_from_buffer(int frame_number, std::map<int, DetectorOut::MSG_Detections> *detections_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (detections_buffer_ptr->find(frame_number) != detections_buffer_ptr->end()) {
        detections_buffer_ptr->erase(frame_number);
    }
}

void DetectorOut::_merge_detections_and_documents()
{
    std::vector<decltype(m_document_buffer)::value_type> document_buffer_;

    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        for (auto const &it : (**lock_ptr_document_buffer)) {
            document_buffer_.push_back(it);
        }
    }

    for (auto &it : document_buffer_) {
        auto &document = it.second;
        auto frame_num = it.first;

        MSG_Detections detections;
        bool has_detections = false;
        {
            auto lock_ptr_detections_buffer = m_impl->sync_detections_buffer.synchronize();
            if ((*lock_ptr_detections_buffer)->find(frame_num) != (*lock_ptr_detections_buffer)->end()) {
                detections = (**lock_ptr_detections_buffer)[frame_num];
                has_detections = true;
            }
        }

        if (!has_detections) {
            continue;
        }

        // RCLCPP_DEBUG(m_impl->logger, "_merge_detections_and_documents for frame %d", frame_num);
        // RCLCPP_DEBUG(m_impl->logger, "_merge document %ld with UUID %s",
        //              document.frame.frame_num, uuid_to_string(document.detections_uuid.uuid).c_str());
        // RCLCPP_DEBUG(m_impl->logger, "_merge detections %ld with UUID %s",
        //              detections.frame.frame_num, uuid_to_string(detections.uuid.uuid).c_str());

        if (document.detections_uuid == detections.uuid) {
            RCLCPP_INFO(m_impl->logger, "_merge_detections_and_documents() for frame %d", frame_num);
            // merge detections and documents
            document.detections = detections;
            {
                auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                _process_document_create_tasks(document, *lock_ptr_document_task_waiting);
            }

            // remove detections from buffer
            {
                auto lock_ptr_detections_buffer = m_impl->sync_detections_buffer.synchronize();
                _remove_detections_from_buffer(frame_num, *lock_ptr_detections_buffer);
            }

            // remove documents from buffer
            {
                auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
                _remove_document_from_buffer(frame_num, *lock_ptr_document_buffer);
            }
        }
    }
}
} // namespace FlowRos2Pipeline