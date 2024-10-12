#include <boost/thread/synchronized_value.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <rclcpp/utilities.hpp>

#include <pose_detector/_pipeline.hpp>
#include <pose_detector/pipeline.hpp>
#include <rcpputils/asserts.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
PoseDetectorPipeline::PoseDetectorPipeline()
    : Node("pose_detector_pipeline_node")
{
    m_impl = std::make_shared<PoseDetectorPipelineImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    // m_impl->sync_document_doing_map = &m_psgdoc_task_doing;
    m_impl->sync_detections_waiting_map = &m_detections_task_waiting;
    // m_impl->sync_detections_doing_map = &m_detections_task_doing;
    m_impl->sync_document_buffer = &m_document_buffer;
    m_impl->sync_bodyposes_buffer = &m_bodyposes_buffer;

    // RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int PoseDetectorPipeline::init(const std::shared_ptr<InitConfig> &config,
                               const std::shared_ptr<RuntimeConfig> &runtime_config)
{
    ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::STOPPED,
               "init FAILED! status code is not BEFORE_INIT or STOPPED");

    m_init_config = config;
    m_runtime_config = runtime_config;

    // setup downstreams
    _connect_to_downstreams();

    // create Pipeline server
    m_act_process_document = rclcpp_action::create_server<ACT_AcceptDocument>(
        this, m_init_config->process_document_action,
        std::bind(&PoseDetectorPipeline::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PoseDetectorPipeline::_accept_document_cancel_callback, this, std::placeholders::_1),
        std::bind(&PoseDetectorPipeline::_accept_document_accepted_callback, this, std::placeholders::_1));

    // create Model server
    m_act_accept_model_results = rclcpp_action::create_server<ACT_AcceptBodyposes>(
        this, m_init_config->process_model_results_action,
        std::bind(&PoseDetectorPipeline::_accept_model_results_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PoseDetectorPipeline::_accept_model_results_cancel_callback, this, std::placeholders::_1),
        std::bind(&PoseDetectorPipeline::_accept_model_results_accepted_callback, this, std::placeholders::_1));

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PoseDetectorPipeline::InitConfig> &PoseDetectorPipeline::get_init_config() const
{
    return m_init_config;
}

int PoseDetectorPipeline::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PoseDetectorPipeline::RuntimeConfig> &PoseDetectorPipeline::get_runtime_config() const
{
    return m_runtime_config;
}


int PoseDetectorPipeline::start()
{
    // the node must be opened
    ROS_ASSERT(m_status_code == NodeStatusCode::INITIALIZED,
               "cannot start because status code is not INITIALIZED");

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STARTED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);

    // start step thread
    // _step重复调用，往下游发送数据，包括model node和pipeline node
    m_impl->step_running = true;
    m_impl->step_thread = std::make_shared<std::thread>(
        [this]() {
            while (rclcpp::ok() && m_impl->step_running) {
                _step();
                // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
            }
        });

    // start process thread
    // _process_step重复调用，一直试图将document buffer中的第一个document和其对应的detections buffer中的数据进行合并
    // 并加入到map_document_waiting中
    m_impl->process_thread = std::make_shared<std::thread>(
        [this]() {
            while (rclcpp::ok() && m_impl->step_running) {
                _process_step();
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
            }
        });

    return ReturnCode::SUCCESS;
}

int PoseDetectorPipeline::stop()
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

    if (m_impl->process_thread) {
        m_impl->process_thread->join();
        m_impl->process_thread = nullptr;
    }

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STOPPED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}


int PoseDetectorPipeline::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse PoseDetectorPipeline::_accept_document_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDocument::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PoseDetectorPipeline::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PoseDetectorPipeline::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    const auto &goal = goal_handle->get_goal();
    const auto &control_msg = goal->control_msg;

    // // if buffer is full, reject the frame
    // // 无论是不是丢帧模式（不尝试重复发送），都会在这里被拦截，避免buffer溢出
    // if (m_document_buffer.size() >= m_runtime_config->buffer_size) {
    //     auto result = std::make_shared<ACT_AcceptDocument::Result>();
    //     result->return_msg = "Buffer is full";
    //     result->return_code = ReturnCode::REJECTED;
    //     goal_handle->abort(result);
    //     RCLCPP_INFO(m_impl->logger, "REJECTED!!! Buffer is full");
    //     return;
    // }

    // 当没有reject时，ping一定成功
    if (control_msg.control_signal == 1) {
        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Ping accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
        return;
    }

    // time log
    RCLCPP_INFO(m_impl->logger, "---TIME LOG: framenum %ld node %s type %s time %ld", goal->document.frame.frame_num, "pose_detector", "IN", this->now().nanoseconds());

    // cache the document, copy it for modify it
    auto document = goal->document;
    auto &persons = document.persons;

    // generate uuid for every Person and then generate body detections from person body
    MSG_Detections detections;
    for (size_t i = 0; i < persons.persons.size(); i++) {
        boost::uuids::uuid uuid = boost::uuids::random_generator()();

        auto &person = document.persons.persons[i];
        std::copy(uuid.begin(), uuid.end(), person.uuid.uuid.begin());

        auto &body = person.body;
        if (body.category != -1) { // not empty
            body.uuid = person.uuid;
            detections.detections.push_back(body);
        }
    }
    detections.frame = document.frame;

    // add document to buffer
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        _add_document_to_buffer(document, *lock_ptr_document_buffer);
    }

    // 设定好m_bodyposes_buffer的初始值
    {
        auto lock_ptr_bodyposes_buffer = m_impl->sync_bodyposes_buffer.synchronize();
        (**lock_ptr_bodyposes_buffer)[document.frame.frame_num] = std::vector<MSG_Bodyposes>();
    }

    // create tasks for all model downstreams
    {
        auto lock_ptr_detections_task_waiting = m_impl->sync_detections_waiting_map.synchronize();
        _process_detections_create_tasks(detections, *lock_ptr_detections_task_waiting);
    }

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}

rclcpp_action::GoalResponse PoseDetectorPipeline::_accept_model_results_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptBodyposes::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with bodyposes with frame_num %ld", goal->frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PoseDetectorPipeline::_accept_model_results_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptBodyposes>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PoseDetectorPipeline::_accept_model_results_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptBodyposes>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();
    const auto &control_msg = goal->control_msg;

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

void PoseDetectorPipeline::_process_document_create_tasks(const MSG_PsgDocument &document, Map_Document_Waiting *psgdoc_task_waiting_ptr)
{
    // create tasks of this frame for all downstreams
    for (auto &x : m_pipeline_downstreams) {

        auto task = std::make_shared<DSTask_PsgDocument>();
        task->downstream = x.second;
        task->document = document;
        (*psgdoc_task_waiting_ptr)[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
    }
}

void PoseDetectorPipeline::_process_detections_create_tasks(const MSG_Detections &detections, Map_Detections_Waiting *detections_waiting_map_ptr)
{
    // RCLCPP_INFO(m_impl->logger, "_process_detections_create_tasks(): create frame %ld detections tasks for downstreams", detections.frame.frame_num);

    // create tasks of this frame for all downstreams
    for (auto &x : m_model_downstreams) {
        auto task = std::make_shared<DSTask_Detections>();
        task->downstream = x.second;
        task->detections = detections;

        (*detections_waiting_map_ptr)[std::make_tuple(task->downstream.get(), detections.frame.frame_num)] = task;
    }
}

// add document to buffer
void PoseDetectorPipeline::_add_document_to_buffer(const MSG_PsgDocument &document, std::map<int, MSG_PsgDocument> *document_buffer_ptr)
{
    (*document_buffer_ptr)[document.frame.frame_num] = document;
}

// remove document from buffer
void PoseDetectorPipeline::_remove_document_from_buffer(int frame_number, std::map<int, MSG_PsgDocument> *document_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (document_buffer_ptr->find(frame_number) != document_buffer_ptr->end()) {
        document_buffer_ptr->erase(frame_number);
    }
}

void PoseDetectorPipeline::_add_bodyposes_to_buffer(const MSG_Bodyposes &bodyposes, const int frame_number,
                                                    std::map<int, std::vector<MSG_Bodyposes>> *bodyposes_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (bodyposes_buffer_ptr->find(frame_number) != bodyposes_buffer_ptr->end()) {
        (*bodyposes_buffer_ptr)[frame_number].push_back(bodyposes);
    }
}

void PoseDetectorPipeline::_remove_bodyposes_from_buffer(int frame_number, std::map<int, std::vector<MSG_Bodyposes>> *bodyposes_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (bodyposes_buffer_ptr->find(frame_number) != bodyposes_buffer_ptr->end()) {
        bodyposes_buffer_ptr->erase(frame_number);
    }
}

void PoseDetectorPipeline::_step()
{
    _send_document_to_downstreams();
    _send_detections_to_downstreams();
}

void PoseDetectorPipeline::_process_step()
{
    _merge_bodyposes_and_documents();
}

void PoseDetectorPipeline::_connect_to_downstreams()
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
            //         std::bind(&PoseDetectorPipeline::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&PoseDetectorPipeline::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&PoseDetectorPipeline::process_document_result_callback, this, std::placeholders::_1);

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
            std::string name = it.second.accept_detections_action;
            auto client = rclcpp_action::create_client<ACT_AcceptDetections>(this, name);

            ds->accept_detections = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&PoseDetectorPipeline::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&PoseDetectorPipeline::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&PoseDetectorPipeline::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            // RCLCPP_INFO(m_impl->logger, "waiting for model action server %s", name.c_str());
            client->wait_for_action_server();
            // RCLCPP_INFO(m_impl->logger, "model action server %s is ready", name.c_str());
        }

        m_model_downstreams[it.first] = ds;
    }
}

bool PoseDetectorPipeline::_ping_model(std::shared_ptr<DownstreamModel> ds)
{
    auto goal_msg = ACT_AcceptDetections::Goal();
    goal_msg.control_msg.control_signal = 1; // ping
    goal_msg.control_msg.control_msg = "ping";

    // opt.goal_response_callback = callback;
    auto res = ds->accept_detections->async_send_goal(goal_msg, ds->accept_detections_options);

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

bool PoseDetectorPipeline::_ping_pipeline(std::shared_ptr<DownstreamPipeline> ds)
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

// 发送detections到model node，如果失败则重试，这个过程不能丢任何detections
void PoseDetectorPipeline::_send_detections_to_downstreams()
{
    std::vector<decltype(m_detections_task_waiting)::value_type> detections_task_waiting_;
    {
        auto lock_ptr_detections_task_waiting = m_impl->sync_detections_waiting_map.synchronize();

        for (auto const &it : (**lock_ptr_detections_task_waiting)) {
            detections_task_waiting_.push_back(it);
        }
    }

    std::vector<Map_Detections_Waiting::key_type> tasks_to_remove;

    for (auto &it : detections_task_waiting_) {
        auto &task = it.second;
        auto ds = task->downstream;

        while (true) {
            if (!m_runtime_config->send_goal_retry && task->detections.frame.signal_code == SignalCode::RUN) // not retry need to ping
                if (!_ping_model(ds))
                    continue;

            ACT_AcceptDetections::Goal goal;
            goal.detections = task->detections;

            auto handle = task->downstream->accept_detections->async_send_goal(goal, ds->accept_detections_options);

            // RCLCPP_INFO(m_impl->logger, "[Request Send]framenum: %ld detections", goal.detections.frame.frame_num);

            // add timeout condition
            auto t = (long)m_runtime_config->timeout_ms_send_to_downstream;
            // RCLCPP_INFO(m_impl->logger, "send goal %ld", task->frame.frame_num);
            auto wait_result = handle.wait_for(std::chrono::milliseconds(t));
            // RCLCPP_INFO(m_impl->logger, "after wait send goal %ld, wait_result is %d", task->frame.frame_num, wait_result);
            if (wait_result == std::future_status::ready) {
                auto task_response = handle.get();
                // RCLCPP_INFO(m_impl->logger, "_step frame async_send_goal: %ld SUCCESS", task->detections.frame.frame_num);
                if (task_response != nullptr) {
                    // accepted or executing
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                        task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
                        // 这里状态为这两个不一定代表成功，可能是下游还在处理中，当下游返回aborted前也会是这两个状态
                        // 所以这里需要等待下游返回成功或者aborted
                        bool is_dets_task_done = false;
                        while (true) {
                            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                                // 如果发送成功了，is_dets_task_done为true，跳出检查状态的循环，并让外面去判断是否需要重试发送detections
                                RCLCPP_INFO(m_impl->logger, "detections %ld success because SUCCEED", task->detections.frame.frame_num);
                                is_dets_task_done = true;
                                break;
                            } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING) {
                                // 如果发送失败了，is_dets_task_done为true，跳出检查状态的循环，并让外面去判断是否需要重试发送detections
                                RCLCPP_INFO(m_impl->logger, "detections %ld failed because ABORTED", task->detections.frame.frame_num);
                                is_dets_task_done = false;
                                break;
                            } else {
                                // 其他情况还需要等待状态变化
                                // sleep一些时间再去查询状态
                                // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));

                                // FIXME: 暂时当做发送成功处理
                                is_dets_task_done = true;
                                break;
                            }
                        }
                        if (is_dets_task_done) {
                            // 发送成功了，跳出发送detections的循环
                            tasks_to_remove.push_back(it.first);
                            break;
                        } else {
                            // 发送失败了，重发
                            continue;
                        }
                    }
                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        RCLCPP_INFO(m_impl->logger, "detections %ld success because SUCCEEDED", task->detections.frame.frame_num);
                        tasks_to_remove.push_back(it.first);
                        break;
                    }
                    // rejected
                    else {
                        // retry
                        {
                            auto lock_ptr_detections_task_waiting = m_impl->sync_detections_waiting_map.synchronize();
                            (**lock_ptr_detections_task_waiting)[it.first]->retry_times++;
                        }
                        RCLCPP_INFO(m_impl->logger, "detections %ld retry because REJECTED", task->detections.frame.frame_num);
                        continue;
                    }
                } else { // rejected
                    // retry
                    {
                        auto lock_ptr_detections_task_waiting = m_impl->sync_detections_waiting_map.synchronize();
                        (**lock_ptr_detections_task_waiting)[it.first]->retry_times++;
                    }
                    RCLCPP_INFO(m_impl->logger, "detections %ld retry because REJECTED", task->detections.frame.frame_num);
                    continue;
                }
            } else { // timeout
                // retry
                {
                    auto lock_ptr_detections_task_waiting = m_impl->sync_detections_waiting_map.synchronize();
                    (**lock_ptr_detections_task_waiting)[it.first]->retry_times++;
                }
                RCLCPP_INFO(m_impl->logger, "detections %ld retry because TIMEOUT", task->detections.frame.frame_num);
                continue;
            }
        }

        RCLCPP_INFO(m_impl->logger, "_send_detections_to_downstreams(): detections %ld success because SUCCEED", task->detections.frame.frame_num);
    }

    // 跳出循环了表明所有detections都发送成功了，在detections_task_waiting中删除这些frame
    {
        auto lock_ptr_detections_task_waiting = m_impl->sync_detections_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_detections_task_waiting)->erase(it);
        }
    }
}

void PoseDetectorPipeline::_send_document_to_downstreams()
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
                        // 这里状态为这两个不一定代表成功，可能是下游还在处理中，当下游返回aborted前也会是这两个状态
                        // 所以这里需要等待下游返回成功或者aborted
                        bool is_doc_task_done = false;
                        while (true) {
                            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                                // 如果发送成功了，is_doc_task_done为true，跳出发送document的循环
                                RCLCPP_INFO(m_impl->logger, "document %ld success because SUCCEED", task->document.frame.frame_num);
                                is_doc_task_done = true;
                                break;
                            } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING) {
                                // 如果发送失败了，is_doc_task_done为false，跳出发送document的循环，并让外面去判断是否需要重试发送document
                                RCLCPP_INFO(m_impl->logger, "document %ld failed because ABORTED", task->document.frame.frame_num);
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
                            tasks_to_remove.push_back(it.first);
                            break;
                        } else {
                            // 发送失败了，判断是否需要重发
                            if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) {
                                // 不需要重发，跳出发送doc的循环
                                tasks_to_remove.push_back(it.first);
                                RCLCPP_INFO(m_impl->logger, "document %ld drop because ABORTED", task->document.frame.frame_num);
                                break;
                            } else {
                                // 需要重发，继续发送
                                continue;
                            }
                        }
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        tasks_to_remove.push_back(it.first);
                        RCLCPP_INFO(m_impl->logger, "document %ld success because SUCCEED", task->document.frame.frame_num);
                        break;
                    }
                    // rejected
                    else {
                        if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                            // m_psgdoc_task_done.push_back(task);
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
                        // m_psgdoc_task_done.push_back(task);
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
                    // m_psgdoc_task_done.push_back(task);
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

    // 出现在tasks_to_remove中的task都已经发送成功或被丢弃，从psgdoc_task_waiting中删除这些task
    {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_psgdoc_task_waiting)->erase(it);
        }
    }
}

// 当document和bodyposes都准备好后，将其合并，并加入到map_document_waiting中
// 注意每次只处理第一个document，如果没准备好就直接返回，等待下一次处理
void PoseDetectorPipeline::_merge_bodyposes_and_documents()
{
    // 取第一个document
    MSG_PsgDocument document;
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        if ((*lock_ptr_document_buffer)->empty())
            return;
        document = (*lock_ptr_document_buffer)->begin()->second;
    }

    // 判断是否有对应的bodyposes且是否全都准备好了
    {
        auto lock_ptr_bodyposes_buffer = m_impl->sync_bodyposes_buffer.synchronize();
        if ((*lock_ptr_bodyposes_buffer)->find(document.frame.frame_num) == (*lock_ptr_bodyposes_buffer)->end())
            return;
        if ((*lock_ptr_bodyposes_buffer)->at(document.frame.frame_num).size() != m_model_downstreams.size())
            return;

        RCLCPP_INFO(m_impl->logger, "_merge_bodyposes_and_documents(): frame %d bodyposes MERGED", document.frame.frame_num);

        // 合并document.persons和bodyposes
        auto &persons = document.persons;
        for (auto &bodyposes : (*lock_ptr_bodyposes_buffer)->at(document.frame.frame_num)) { // x: MSG_Bodyposes (std::vector<psg_public_msgs::msg::BodyPose>)
            for (auto &bodypose : bodyposes) {                                               // y: psg_public_msgs::msg::BodyPose 表示一个人的姿态，里面有多个关键点
                for (auto &person : persons.persons) {                                       // z: psg_public_msgs::msg::Person 表示一个人的信息，里面有uuid和body
                    if (bodypose.uuid == person.uuid) {
                        person.pose = bodypose;
                        break;
                    }
                }
            }
        }
    }

    // 加入到map_document_waiting中
    {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        _process_document_create_tasks(document, *lock_ptr_psgdoc_task_waiting);
    }

    // 从buffer中删除document
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        _remove_document_from_buffer(document.frame.frame_num, *lock_ptr_document_buffer);
    }

    // 从buffer中删除bodyposes
    {
        auto lock_ptr_bodyposes_buffer = m_impl->sync_bodyposes_buffer.synchronize();
        _remove_bodyposes_from_buffer(document.frame.frame_num, *lock_ptr_bodyposes_buffer);
    }
}

void PoseDetectorPipeline::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<std::string>("process_model_results_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
    this->declare_parameter<int>("buffer_size", 1);
    this->declare_parameter<bool>("send_goal_retry", false);
}
} // namespace FlowRos2Pipeline