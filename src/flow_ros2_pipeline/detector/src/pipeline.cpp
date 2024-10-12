#include <boost/thread/synchronized_value.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <rclcpp/utilities.hpp>

#include <detector/_pipeline.hpp>
#include <detector/pipeline.hpp>
#include <rcpputils/asserts.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
std::string bbox_msg_to_string(const psg_public_msgs::msg::Box2 &bbox)
{
    std::string str = "(";
    str += std::to_string(bbox.x) + ", ";
    str += std::to_string(bbox.y) + ", ";
    str += std::to_string(bbox.width) + ", ";
    str += std::to_string(bbox.height) + ")";

    return str;
}


std::string detection_msg_to_string(const DetectorPipeline::MSG_Detection &detection)
{
    std::string str = "Detection: {\n";
    str += "frame_num: " + std::to_string(detection.frame.frame_num) + "\n";
    str += "category: " + std::to_string(detection.category) + "\n";
    str += "uuid: " + uuid_to_string(detection.uuid.uuid) + "\n";
    str += "bbox: " + bbox_msg_to_string(detection.bbox) + "\n";
    str += "confidence: " + std::to_string(detection.confidence) + "\n";
    str += "}";

    return str;
}

DetectorPipeline::DetectorPipeline()
    : Node("detector_pipeline_node")
{
    m_impl = std::make_shared<DetectorPipelineImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    // m_impl->sync_document_doing_map = &m_psgdoc_task_doing;
    m_impl->sync_frame_waiting_map = &m_frame_task_waiting;
    // m_impl->sync_frame_doing_map = &m_frame_task_doing;
    m_impl->sync_document_buffer = &m_document_buffer;
    m_impl->sync_detections_buffer = &m_detections_buffer;

    // RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int DetectorPipeline::init(const std::shared_ptr<InitConfig> &config,
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
        std::bind(&DetectorPipeline::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DetectorPipeline::_accept_document_cancel_callback, this, std::placeholders::_1),
        std::bind(&DetectorPipeline::_accept_document_accepted_callback, this, std::placeholders::_1));

    // create Model server
    m_act_accept_model_results = rclcpp_action::create_server<ACT_AcceptDetections>(
        this, m_init_config->process_model_results_action,
        std::bind(&DetectorPipeline::_accept_model_results_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DetectorPipeline::_accept_model_results_cancel_callback, this, std::placeholders::_1),
        std::bind(&DetectorPipeline::_accept_model_results_accepted_callback, this, std::placeholders::_1));

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<DetectorPipeline::InitConfig> &DetectorPipeline::get_init_config() const
{
    return m_init_config;
}

int DetectorPipeline::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<DetectorPipeline::RuntimeConfig> &DetectorPipeline::get_runtime_config() const
{
    return m_runtime_config;
}


int DetectorPipeline::start()
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

int DetectorPipeline::stop()
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


int DetectorPipeline::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse DetectorPipeline::_accept_document_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDocument::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DetectorPipeline::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void DetectorPipeline::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    const auto &goal = goal_handle->get_goal();
    const auto &control_msg = goal->control_msg;

    // buffer size log
    RCLCPP_INFO(m_impl->logger, "frame %d document buffer size: %d", goal->document.frame.frame_num, m_document_buffer.size());

    // if buffer is full, reject the frame
    // 只有丢帧模式（不尝试重复发送），才会在这里被拦截，避免buffer溢出
    if (!m_runtime_config->send_goal_retry && m_document_buffer.size() >= m_runtime_config->buffer_size) {
        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Buffer is full";
        result->return_code = ReturnCode::REJECTED;
        goal_handle->abort(result);
        RCLCPP_INFO(m_impl->logger, "REJECTED!!! Buffer is full");
        return;
    }

    // 当没有reject时，ping一定成功
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

    // add to buffer
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        _add_document_to_buffer(document, *lock_ptr_document_buffer);
    }

    // 设定好m_detections_buffer的初始值
    {
        auto lock_ptr_detections_buffer = m_impl->sync_detections_buffer.synchronize();
        (**lock_ptr_detections_buffer)[frame.frame_num] = std::vector<MSG_Detections>();
    }

    // create tasks for all model downstreams
    {
        auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
        _process_frame_create_tasks(frame, document.detections_uuid, *lock_ptr_frame_task_waiting);
    }

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}

rclcpp_action::GoalResponse DetectorPipeline::_accept_model_results_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDetections::Goal> goal)
{
    // RCLCPP_INFO(m_impl->logger, "Received goal request with detections with frame_num %ld", goal->detections.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse DetectorPipeline::_accept_model_results_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}


void DetectorPipeline::_accept_model_results_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle)
{
    const auto &goal = goal_handle->get_goal();
    const auto &control_msg = goal->control_msg;

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


void DetectorPipeline::_process_document_create_tasks(const MSG_PsgDocument &document, Map_Document_Waiting *psgdoc_task_waiting_ptr)
{
    // create tasks of this frame for all downstreams
    for (auto &x : m_pipeline_downstreams) {

        auto task = std::make_shared<DSTask_PsgDocument>();
        task->downstream = x.second;
        task->document = document;
        (*psgdoc_task_waiting_ptr)[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
    }
}

void DetectorPipeline::_process_frame_create_tasks(const MSG_Frame &frame, const MSG_UUID &detections_uuid, Map_Frame_Waiting *frame_waiting_map_ptr)
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

// add document to buffer
void DetectorPipeline::_add_document_to_buffer(const MSG_PsgDocument &document, std::map<int, MSG_PsgDocument> *document_buffer_ptr)
{
    (*document_buffer_ptr)[document.frame.frame_num] = document;
}

// remove document from buffer
void DetectorPipeline::_remove_document_from_buffer(int frame_number, std::map<int, MSG_PsgDocument> *document_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (document_buffer_ptr->find(frame_number) != document_buffer_ptr->end()) {
        document_buffer_ptr->erase(frame_number);
    }
}

// add detections to buffer
void DetectorPipeline::_add_detections_to_buffer(const MSG_Detections &detections, std::map<int, std::vector<MSG_Detections>> *detections_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (detections_buffer_ptr->find(detections.frame.frame_num) != detections_buffer_ptr->end()) {
        (*detections_buffer_ptr)[detections.frame.frame_num].push_back(detections);
    }
}

// remove detections from buffer
void DetectorPipeline::_remove_detections_from_buffer(int frame_number, std::map<int, std::vector<MSG_Detections>> *detections_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (detections_buffer_ptr->find(frame_number) != detections_buffer_ptr->end()) {
        detections_buffer_ptr->erase(frame_number);
    }
}

void DetectorPipeline::_step()
{
    _send_document_to_downstreams();
    _send_frame_to_downstreams();
}

void DetectorPipeline::_process_step()
{
    _merge_detections_and_documents();
}

void DetectorPipeline::_connect_to_downstreams()
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
            //         std::bind(&DetectorPipeline::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&DetectorPipeline::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&DetectorPipeline::process_document_result_callback, this, std::placeholders::_1);

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
            //         std::bind(&DetectorPipeline::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&DetectorPipeline::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&DetectorPipeline::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            // RCLCPP_INFO(m_impl->logger, "waiting for model action server %s", name.c_str());
            client->wait_for_action_server();
            // RCLCPP_INFO(m_impl->logger, "model action server %s is ready", name.c_str());
        }

        m_model_downstreams[it.first] = ds;
    }
}

bool DetectorPipeline::_ping_model(std::shared_ptr<DownstreamModel> ds)
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

bool DetectorPipeline::_ping_pipeline(std::shared_ptr<DownstreamPipeline> ds)
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

// 发送frame到model node，如果失败则重试，这个过程不能丢任何frame
void DetectorPipeline::_send_frame_to_downstreams()
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
                        // 这里状态为这两个不一定代表成功，可能是下游还在处理中，当下游返回aborted前也会是这两个状态
                        // 所以这里需要等待下游返回成功或者aborted
                        bool is_frame_task_done = false;
                        while (true) {
                            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                                // 如果发送成功了，is_frame_task_done为true，跳出检查状态的循环，并让外面去判断是否需要重试发送frame
                                RCLCPP_INFO(m_impl->logger, "frame %ld success because SUCCEED", task->frame.frame_num);
                                is_frame_task_done = true;
                                break;
                            } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING) {
                                // 如果发送失败了，is_frame_task_done为false，跳出检查状态的循环，并让外面去判断是否需要重试发送frame
                                RCLCPP_INFO(m_impl->logger, "frame %ld failed because ABORTED", task->frame.frame_num);
                                is_frame_task_done = false;
                                break;
                            } else {
                                // 其他情况还需要等待状态变化
                                // sleep一些时间再去查询状态
                                // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));

                                // FIXME: 暂时当做发送成功处理
                                is_frame_task_done = true;
                                break;
                            }
                        }
                        if (is_frame_task_done) {
                            // 发送成功了，跳出发送frame的循环
                            tasks_to_remove.push_back(it.first);
                            break;
                        } else {
                            // 发送失败了，重发
                            continue;
                        }
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        RCLCPP_INFO(m_impl->logger, "frame %ld success because SUCCEEDED", task->frame.frame_num);
                        tasks_to_remove.push_back(it.first);
                        break;
                    }
                    // rejected
                    else {
                        // retry
                        {
                            auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
                            (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                        }
                        RCLCPP_INFO(m_impl->logger, "frame %ld retry because REJECTED", task->frame.frame_num);
                        continue;
                    }
                } else { // rejected
                    // retry
                    {
                        auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
                        (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                    }
                    RCLCPP_INFO(m_impl->logger, "frame %ld retry because REJECTED", task->frame.frame_num);
                    continue;
                }
            } else { // timeout
                // retry
                {
                    auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
                    (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                }
                RCLCPP_INFO(m_impl->logger, "frame %ld retry because TIMEOUT", task->frame.frame_num);
                continue;
            }
        }
    }


    // 跳出循环了表明所有frame都发送成功了，在frame_task_waiting中删除这些frame
    {
        auto lock_ptr_frame_task_waiting = m_impl->sync_frame_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_frame_task_waiting)->erase(it);
        }
    }
}


void DetectorPipeline::_send_document_to_downstreams()
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

// 当document和detections都准备好后，将其合并，并加入到map_document_waiting中
// 注意每次只处理第一个document，如果没准备好就直接返回，等待下一次处理
void DetectorPipeline::_merge_detections_and_documents()
{
    // 取第一个document
    MSG_PsgDocument document;
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        if ((*lock_ptr_document_buffer)->empty())
            return;
        document = (*lock_ptr_document_buffer)->begin()->second;
    }

    document.detections.frame = document.frame;

    // 判断是否有对应的detections且是否全都准备好了
    {
        auto lock_ptr_detections_buffer = m_impl->sync_detections_buffer.synchronize();
        if ((*lock_ptr_detections_buffer)->find(document.frame.frame_num) == (*lock_ptr_detections_buffer)->end())
            return;
        if ((*lock_ptr_detections_buffer)->at(document.frame.frame_num).size() != m_model_downstreams.size())
            return;

        RCLCPP_INFO(m_impl->logger, "_merge_detections_and_documents(): frame %d detections MERGED", document.frame.frame_num);

        // 合并document和detections
        for (auto &x : (*lock_ptr_detections_buffer)->at(document.frame.frame_num)) {
            document.detections.detections.insert(document.detections.detections.end(), x.detections.begin(), x.detections.end());
        }
    }

    // // test log
    // for (const auto &detection : document.detections.detections) {
    //     RCLCPP_INFO(m_impl->logger, "_merge_detections_and_documents(): detection %s",
    //                 detection_msg_to_string(detection).c_str());
    // }

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

    // 从buffer中删除detections
    {
        auto lock_ptr_detections_buffer = m_impl->sync_detections_buffer.synchronize();
        _remove_detections_from_buffer(document.frame.frame_num, *lock_ptr_detections_buffer);
    }
}


void DetectorPipeline::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<std::string>("process_model_results_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", 10000);
    this->declare_parameter<int>("buffer_size", 1);
    this->declare_parameter<bool>("send_goal_retry", false);
}
} // namespace FlowRos2Pipeline