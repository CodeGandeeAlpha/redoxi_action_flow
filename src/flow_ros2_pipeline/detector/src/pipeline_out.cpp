#include "psg_common/psg_common.hpp"
#include <boost/thread/lock_algorithms.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <set>

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

    RCLCPP_INFO(m_impl->logger, "constraction success!");
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

    // // setup downstreams
    // _connect_to_downstreams();

    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                m_status_code, NodeStatusCode::INITIALIZED);
    m_status_code = NodeStatusCode::INITIALIZED;
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

    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                m_status_code, NodeStatusCode::STARTED);

    m_status_code = NodeStatusCode::STARTED;

    m_impl->step_running = true;
    m_impl->step_thread = std::make_shared<std::thread>(
        [this]() {
            while (rclcpp::ok() && m_impl->step_running) {
                _step();
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
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

    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                m_status_code, NodeStatusCode::STOPPED);

    m_status_code = NodeStatusCode::STOPPED;
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
    RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DetectorOut::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

std::string uuid_to_string(const std::array<uint8_t, 16> &uuid)
{
    // std::ostringstream oss;
    // for (const auto& byte : uuid) {
    //     oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    // }
    // return oss.str();

    std::ostringstream oss;
    for (size_t i = 0; i < uuid.size(); ++i) {
        if (i != 0) {
            oss << "-";
        }
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(uuid[i]);
    }
    return oss.str();
}


void DetectorOut::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the document
    const auto &document = goal->document;

    // add to buffer
    _add_document_to_buffer(document);

    RCLCPP_INFO(m_impl->logger, "Accepted document %ld and add it to buffer", document.frame.frame_num);
    RCLCPP_INFO(m_impl->logger, "Accepted document %ld with UUID %s and add it to buffer",
                document.frame.frame_num, uuid_to_string(document.uuid.uuid).c_str());

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}

rclcpp_action::GoalResponse DetectorOut::_accept_detections_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDetections::Goal> goal)
{
    RCLCPP_INFO(m_impl->logger, "Received goal request with detections with frame_num %ld", goal->detections.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DetectorOut::_accept_detections_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void DetectorOut::_accept_detections_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the detections
    const auto &detections = goal->detections;

    // add to buffer
    _add_detections_to_buffer(detections);

    RCLCPP_INFO(m_impl->logger, "add detections to buffer");

    auto result = std::make_shared<ACT_AcceptDetections::Result>();
    result->return_msg = "Detections accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}


void DetectorOut::_process_document_create_tasks(const MSG_PsgDocument &document,
                                                 DetectorOut::Map_Document_Waiting *document_waiting_map_ptr)
{
    RCLCPP_INFO(m_impl->logger, "create tasks for document %ld", document.frame.frame_num);

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
    // _send_document_to_downstreams();
}

void DetectorOut::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

    m_downstreams.clear();

    for (auto it : m_init_config->downstreams) {
        auto ds = std::make_shared<Downstream>();
        RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

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
            RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_downstreams[it.first] = ds;
    }
}

void DetectorOut::_send_document_to_downstreams()
{
    std::set<int> useful_documents;
    std::vector<int> useless_documents;

    {
        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.unique_synchronize();
        auto lock_ptr_document_task_doing = m_impl->sync_document_doing_map.unique_synchronize();
        boost::lock(lock_ptr_document_task_waiting, lock_ptr_document_task_doing);
        if (!(*lock_ptr_document_task_waiting)->empty()) {
            // initiate all waiting tasks
            std::vector<Map_Document_Waiting::key_type> tasks_to_remove;

            for (auto &it : (**lock_ptr_document_task_waiting)) {
                auto &task = it.second;
                ACT_AcceptDocument::Goal goal;
                goal.document = task->document;
                auto ds = task->downstream;
                auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);

                // FIXME: add timeout condition
                auto task_response = handle.get();
                if (task_response != nullptr) {
                    // accepted
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
                        // successfully sent, record this
                        task->goal_handle = task_response;
                        // task->status = DSTask_PSGDocument::TASK_SENT;
                        (**lock_ptr_document_task_doing)[task->goal_handle] = task;
                        tasks_to_remove.push_back(it.first);
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        // task->status = DSTask_PSGDocument::TASK_DONE;
                        tasks_to_remove.push_back(it.first);
                    }
                }
                // else {
                //     // rejected
                //     task->status = DSTask_PSGDocument::TASK_FAILED;
                // }

                // FIXME: what if failed to send many times?
                // you need to terminate a frame, remove it from memory registry
            }

            // remove all sent tasks
            for (auto &it : tasks_to_remove) {
                (*lock_ptr_document_task_waiting)->erase(it);
            }
        }

        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_document_task_doing)->empty()) {
            std::vector<GoalHandle_PsgDocument> tasks_to_remove;
            for (auto &it : (**lock_ptr_document_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_document_task_doing)->erase(it);
            }
        }

        // if a frame has no waiting tasks or running tasks associated with it, remove it from buffer
        for (auto &it : (**lock_ptr_document_task_waiting)) {
            useful_documents.insert(it.second->document.frame.frame_num);
        }
        for (auto &it : (**lock_ptr_document_task_doing)) {
            useful_documents.insert(it.second->document.frame.frame_num);
        }
    }

    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        for (auto &it : (**lock_ptr_document_buffer)) {
            // if not useful, it is useless
            if (useful_documents.find(it.first) == useful_documents.end()) {
                useless_documents.push_back(it.first);
            }
        }
        // remove useless documents
        for (auto &it : useless_documents) {
            _remove_document_from_buffer(it, *lock_ptr_document_buffer);
        }
    }
}


void DetectorOut::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<std::string>("process_detections_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
}


void DetectorOut::_add_document_to_buffer(const MSG_PsgDocument &document)
{
    auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
    (**lock_ptr_document_buffer)[document.frame.frame_num] = document;
}

void DetectorOut::_add_detections_to_buffer(const MSG_Detections &detections)
{
    auto lock_ptr_detections_buffer = m_impl->sync_detections_buffer.synchronize();
    (**lock_ptr_detections_buffer)[detections.frame.frame_num] = detections;
}

void DetectorOut::_remove_document_from_buffer(int frame_number, std::map<int, DetectorOut::MSG_PsgDocument> *document_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (document_buffer_ptr->find(frame_number) != document_buffer_ptr->end()) {
        document_buffer_ptr->erase(frame_number);
    }
}

void DetectorOut::_merge_detections_and_documents()
{
    auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.unique_synchronize();
    auto lock_ptr_document_buffer = m_impl->sync_document_buffer.unique_synchronize();
    auto lock_ptr_detections_buffer = m_impl->sync_detections_buffer.unique_synchronize();
    boost::lock(lock_ptr_document_buffer, lock_ptr_detections_buffer);

    for (auto &it : (**lock_ptr_document_buffer)) {
        auto &document = it.second;
        auto frame_num = it.first;
        if ((*lock_ptr_detections_buffer)->find(frame_num) != (*lock_ptr_detections_buffer)->end()) {
            auto &detections = (**lock_ptr_detections_buffer)[frame_num];

            RCLCPP_INFO(m_impl->logger, "_merge_detections_and_documents for frame %d", frame_num);
            RCLCPP_INFO(m_impl->logger, "_merge document %ld with UUID %s and add it to buffer",
                        document.frame.frame_num, uuid_to_string(document.detections_uuid.uuid).c_str());
            RCLCPP_INFO(m_impl->logger, "_merge detections %ld with UUID %s and add it to buffer",
                        document.frame.frame_num, uuid_to_string(detections.uuid.uuid).c_str());

            if (document.detections_uuid == detections.uuid) {
                // merge detections and documents
                document.detections = detections;
                _process_document_create_tasks(document, *lock_ptr_document_task_waiting);

                // remove detections from buffer
                (*lock_ptr_detections_buffer)->erase(frame_num);
            }
        }
    }
}
} // namespace FlowRos2Pipeline