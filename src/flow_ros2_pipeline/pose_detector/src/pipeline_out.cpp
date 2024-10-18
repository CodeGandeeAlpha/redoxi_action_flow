#include <boost/thread/lock_algorithms.hpp>
#include <psg_common/psg_common.hpp>

#include <pose_detector/_pipeline_out.hpp>
#include <pose_detector/pipeline_out.hpp>
#include <rcpputils/asserts.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
PoseDetectorOut::PoseDetectorOut()
    : Node("pose_detector_out_node")
{
    m_impl = std::make_shared<PoseDetectorOutImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    m_impl->sync_document_doing_map = &m_psgdoc_task_doing;

    m_impl->sync_document_buffer = &m_document_buffer;
    m_impl->sync_bodyposes_buffer = &m_bodyposes_buffer;


    // RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int PoseDetectorOut::init(const std::shared_ptr<InitConfig> &config,
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
        std::bind(&PoseDetectorOut::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PoseDetectorOut::_accept_document_cancel_callback, this, std::placeholders::_1),
        std::bind(&PoseDetectorOut::_accept_document_accepted_callback, this, std::placeholders::_1));

    // create process bodyposes server
    // std::string process_detections_action = this->get_parameter(m_init_config->process_detections_action).as_string();
    m_act_process_bodyposes = rclcpp_action::create_server<ACT_AcceptBodyposes>(
        this, m_init_config->process_bodyposes_action,
        std::bind(&PoseDetectorOut::_accept_bodyposes_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PoseDetectorOut::_accept_bodyposes_cancel_callback, this, std::placeholders::_1),
        std::bind(&PoseDetectorOut::_accept_bodyposes_accepted_callback, this, std::placeholders::_1));

    // setup downstreams
    _connect_to_downstreams();

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PoseDetectorOut::InitConfig> &PoseDetectorOut::get_init_config() const
{
    return m_init_config;
}

int PoseDetectorOut::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PoseDetectorOut::RuntimeConfig> &PoseDetectorOut::get_runtime_config() const
{
    return m_runtime_config;
}


int PoseDetectorOut::start()
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

int PoseDetectorOut::stop()
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


int PoseDetectorOut::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse PoseDetectorOut::_accept_document_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDocument::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PoseDetectorOut::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PoseDetectorOut::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();
    const auto &x_control = goal->x_control;

    // // if buffer is full, reject the frame
    // if (m_document_buffer.size() >= m_runtime_config->buffer_size) {
    //     auto result = std::make_shared<ACT_AcceptDocument::Result>();
    //     result->return_msg = "Buffer is full";
    //     result->return_code = ReturnCode::REJECTED;
    //     goal_handle->abort(result);
    //     return;
    // }

    // ping
    if (x_control.code == 1) {
        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Ping accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
        return;
    }

    // cache the document
    const auto &document = goal->document;

    // add to buffer
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        _add_document_to_buffer(document, *lock_ptr_document_buffer);

        // RCLCPP_INFO(m_impl->logger, "Accepted document %ld with UUID %s and add it to buffer",
        // document.frame.frame_num, uuid_to_string(document.x_uid.uuid).c_str());
    }

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}

rclcpp_action::GoalResponse PoseDetectorOut::_accept_bodyposes_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptBodyposes::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with bodyposes with frame_num %ld", goal->frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PoseDetectorOut::_accept_bodyposes_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptBodyposes>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PoseDetectorOut::_accept_bodyposes_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptBodyposes>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();
    const auto &x_control = goal->x_control;

    // // if buffer is full, reject the frame
    // if (m_bodyposes_buffer.size() >= m_runtime_config->buffer_size) {
    //     auto result = std::make_shared<ACT_AcceptBodyposes::Result>();
    //     result->return_msg = "Buffer is full";
    //     result->return_code = ReturnCode::REJECTED;
    //     goal_handle->abort(result);
    //     return;
    // }

    // ping
    if (x_control.code == 1) {
        auto result = std::make_shared<ACT_AcceptBodyposes::Result>();
        result->return_msg = "Ping accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
        return;
    }

    // cache the bodyposes
    const auto &body_poses = goal->body_poses;
    const auto &frame = goal->frame;

    // add to buffer
    {
        auto lock_ptr_bodyposes_buffer = m_impl->sync_bodyposes_buffer.synchronize();
        _add_bodyposes_to_buffer(body_poses, frame.frame_num, *lock_ptr_bodyposes_buffer);
    }

    // RCLCPP_INFO(m_impl->logger, "_accept_bodyposes_accepted_callback(): Accepted frame_number %ld and add it to buffer", frame.frame_num);

    auto result = std::make_shared<ACT_AcceptBodyposes::Result>();
    result->return_msg = "Bodyposes accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}


void PoseDetectorOut::_process_document_create_tasks(const MSG_PsgDocument &document,
                                                     PoseDetectorOut::Map_Document_Waiting *document_waiting_map_ptr)
{
    // RCLCPP_INFO(m_impl->logger, "_process_document_create_tasks(): create tasks for document %ld", document.frame.frame_num);

    // create tasks of this frame for all downstreams
    for (auto &x : m_downstreams) {
        auto task = std::make_shared<DSTask_PsgDocument>();
        task->downstream = x.second;
        task->document = document;
        (*document_waiting_map_ptr)[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
    }
}


void PoseDetectorOut::_step()
{
    _merge_bodyposes_and_documents();
    _send_document_to_downstreams();
}

void PoseDetectorOut::_connect_to_downstreams()
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
            //         std::bind(&PoseDetectorOut::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&PoseDetectorOut::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&PoseDetectorOut::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            // RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            // RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_downstreams[it.first] = ds;
    }
}

bool PoseDetectorOut::_ping(const std::shared_ptr<Downstream> &ds)
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

void PoseDetectorOut::_send_document_to_downstreams()
{
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
        goal.document = task->document;
        auto ds = task->downstream;

        while (true) {
            if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) // not retry need to ping
                if (!_ping(ds))
                    continue;

            // time log
            RCLCPP_INFO(m_impl->logger, "---TIME LOG: framenum %ld node %s type %s time %ld", goal.document.frame.frame_num, "pose_detector", "OUT", this->now().nanoseconds());

            auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);

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
                        // task->status = DSTask_PSGDocument::TASK_SENT;
                        {
                            auto lock_ptr_document_task_doing = m_impl->sync_document_doing_map.synchronize();
                            (**lock_ptr_document_task_doing)[task->goal_handle] = task;
                        }
                        tasks_to_remove.push_back(it.first);
                        break;
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        // task->status = DSTask_PSGDocument::TASK_DONE;
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
                    }
                } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED) {
                    if (!m_runtime_config->send_goal_retry && it.second->document.frame.signal_code == SignalCode::RUN) { // failed
                        m_psgdoc_task_done.push_back(it.second);
                        tasks_to_remove.push_back(it.first);
                    } else {
                        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                        (**lock_ptr_document_task_waiting)[std::make_tuple(it.second->downstream.get(),
                                                                           it.second->document.frame.frame_num)] = it.second;

                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_document_task_doing)->erase(it);
            }
        }
    }

    // {
    //     auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
    //     // remove task done documents
    //     for (auto &it : m_psgdoc_task_done) {
    //         _remove_document_from_buffer(it->document.frame.frame_num, *lock_ptr_document_buffer);
    //     }
    // }

    // for all done tasks, remove them from memory
    m_psgdoc_task_done.clear();
}


void PoseDetectorOut::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<std::string>("process_bodyposes_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
    this->declare_parameter<int>("buffer_size", 1);
    this->declare_parameter<bool>("send_goal_retry", false);
}


void PoseDetectorOut::_add_document_to_buffer(const MSG_PsgDocument &document, std::map<int, PoseDetectorOut::MSG_PsgDocument> *document_buffer_ptr)
{
    (*document_buffer_ptr)[document.frame.frame_num] = document;
}

void PoseDetectorOut::_add_bodyposes_to_buffer(const MSG_Bodyposes &bodyposes, const int frame_number, std::map<int, PoseDetectorOut::MSG_Bodyposes> *bodyposes_buffer_ptr)
{
    (*bodyposes_buffer_ptr)[frame_number] = bodyposes;
}

void PoseDetectorOut::_remove_document_from_buffer(int frame_number, std::map<int, PoseDetectorOut::MSG_PsgDocument> *document_buffer_ptr)
{
    // RCLCPP_DEBUG(m_impl->logger, "_remove_document_from_buffer(): remove document with frame_num %d", frame_number);
    // if frame_number is not in buffer, do nothing
    if (document_buffer_ptr->find(frame_number) != document_buffer_ptr->end()) {
        document_buffer_ptr->erase(frame_number);
        // RCLCPP_DEBUG(m_impl->logger, "_remove_document_from_buffer(): remove document with frame_num %d SUCCESS", frame_number);
    }
}

void PoseDetectorOut::_remove_bodyposes_from_buffer(int frame_number, std::map<int, PoseDetectorOut::MSG_Bodyposes> *bodyposes_buffer_ptr)
{
    // RCLCPP_DEBUG(m_impl->logger, "_remove_bodyposes_from_buffer(): remove bodyposes with frame_num %d", frame_number);
    // if frame_number is not in buffer, do nothing
    if (bodyposes_buffer_ptr->find(frame_number) != bodyposes_buffer_ptr->end()) {
        bodyposes_buffer_ptr->erase(frame_number);
        // RCLCPP_DEBUG(m_impl->logger, "_remove_bodyposes_from_buffer(): remove bodyposes with frame_num %d SUCCESS", frame_number);
    }
}

/**
 * @brief 合并 bodyposes 和 documents。
 *
 * 该函数从文档缓冲区中提取文档，并尝试将其与 bodyposes 缓冲区中的 bodyposes 合并。
 * 如果文档的信号代码为 FLUSH 或 TERMINATE，则创建文档任务并从缓冲区中移除相应的 bodyposes 和文档。
 * 否则，将 bodyposes 与文档中的人员信息合并，并创建文档任务，然后从缓冲区中移除相应的 bodyposes 和文档。
 *
 * @note 该函数使用多个同步锁来确保线程安全。
 */
void PoseDetectorOut::_merge_bodyposes_and_documents()
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
        const auto frame_num = it.first;

        MSG_Bodyposes bodyposes;
        bool has_bodyposes = false;
        {
            auto lock_ptr_bodyposes_buffer = m_impl->sync_bodyposes_buffer.synchronize();
            // // test
            // for (auto const &it : (**lock_ptr_bodyposes_buffer)) {
            //     //RCLCPP_INFO(m_impl->logger, "_merge_bodyposes_and_documents(): bodyposes_buffer frame_num %d", it.first);
            // }

            if ((*lock_ptr_bodyposes_buffer)->find(frame_num) != (*lock_ptr_bodyposes_buffer)->end()) {
                bodyposes = (**lock_ptr_bodyposes_buffer)[frame_num];
                has_bodyposes = true;
            }
        }

        if (!has_bodyposes) {
            continue;
        }

        // RCLCPP_DEBUG(m_impl->logger, "_merge_bodyposes_and_documents(): for frame %d", frame_num);
        // RCLCPP_DEBUG(m_impl->logger, "_merge_bodyposes_and_documents(): _merge framenum %ld document and bodyposes", document.frame.frame_num);

        // after bodypose in buffer, if signal code is FLUSH OR TERMINATE
        if (document.signal_code == SignalCode::FLUSH || document.signal_code == SignalCode::TERMINATE) {
            // create task for document
            {
                auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                _process_document_create_tasks(document, *lock_ptr_document_task_waiting);
            }

            // remove bodypose from buffer
            {
                auto lock_ptr_bodyposes_buffer = m_impl->sync_bodyposes_buffer.synchronize();
                _remove_bodyposes_from_buffer(frame_num, *lock_ptr_bodyposes_buffer);
            }

            // remove document from buffer
            {
                auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
                _remove_document_from_buffer(frame_num, *lock_ptr_document_buffer);
            }
            continue;
        }


        // merge bodyposes and document's persons
        // bool is_merged = false;
        for (auto &bodypose : bodyposes) {
            // find the corresponding persons
            auto &persons = document.persons;
            for (auto &person : persons.persons) {
                if (bodypose.uuid == person.uuid) {
                    // merge bodypose and person
                    person.pose = bodypose;
                    // is_merged = true;
                    // //RCLCPP_INFO(m_impl->logger, "_merge_bodyposes_and_documents(): merged bodypose and person with uuid %s", uuid_to_string(person.uuid.uuid).c_str());
                    break;
                }
            }
        }

        // if (is_merged)
        {
            // create task for document
            {
                auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                _process_document_create_tasks(document, *lock_ptr_document_task_waiting);
            }

            // remove bodypose from buffer
            {
                auto lock_ptr_bodyposes_buffer = m_impl->sync_bodyposes_buffer.synchronize();
                _remove_bodyposes_from_buffer(frame_num, *lock_ptr_bodyposes_buffer);
            }

            // remove document from buffer
            {
                auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
                _remove_document_from_buffer(frame_num, *lock_ptr_document_buffer);
            }
        }
    }
}
} // namespace FlowRos2Pipeline