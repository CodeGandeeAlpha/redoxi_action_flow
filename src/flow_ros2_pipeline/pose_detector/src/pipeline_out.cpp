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


    RCLCPP_INFO(m_impl->logger, "constraction success!");
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

    // create process detections server
    // std::string process_detections_action = this->get_parameter(m_init_config->process_detections_action).as_string();
    m_act_process_bodyposes = rclcpp_action::create_server<ACT_AcceptBodyposes>(
        this, m_init_config->process_bodyposes_action,
        std::bind(&PoseDetectorOut::_accept_bodyposes_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PoseDetectorOut::_accept_bodyposes_cancel_callback, this, std::placeholders::_1),
        std::bind(&PoseDetectorOut::_accept_bodyposes_accepted_callback, this, std::placeholders::_1));

    // setup downstreams
    // _connect_to_downstreams();

    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                m_status_code, NodeStatusCode::INITIALIZED);
    m_status_code = NodeStatusCode::INITIALIZED;
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

    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                m_status_code, NodeStatusCode::STOPPED);

    m_status_code = NodeStatusCode::STOPPED;
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
    RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PoseDetectorOut::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PoseDetectorOut::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the document
    const auto &document = goal->document;

    // add to buffer
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        _add_document_to_buffer(document, *lock_ptr_document_buffer);
    }

    RCLCPP_INFO(m_impl->logger, "Accepted document %ld with UUID %s and add it to buffer",
                document.frame.frame_num, uuid_to_string(document.detections_uuid.uuid).c_str());

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}

rclcpp_action::GoalResponse PoseDetectorOut::_accept_bodyposes_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptBodyposes::Goal> goal)
{
    RCLCPP_INFO(m_impl->logger, "Received goal request with detections with frame_num %ld", goal->frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PoseDetectorOut::_accept_bodyposes_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptBodyposes>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PoseDetectorOut::_accept_bodyposes_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptBodyposes>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the detections
    const auto &body_poses = goal->body_poses;
    const auto &frame = goal->frame;

    // add to buffer
    {
        auto lock_ptr_bodyposes_buffer = m_impl->sync_bodyposes_buffer.synchronize();
        _add_bodyposes_to_buffer(body_poses, frame.frame_num, *lock_ptr_bodyposes_buffer);
    }

    RCLCPP_INFO(m_impl->logger, "_accept_bodyposes_accepted_callback(): Accepted frame_number %ld and add it to buffer", frame.frame_num);

    auto result = std::make_shared<ACT_AcceptBodyposes::Result>();
    result->return_msg = "Bodyposes accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}


void PoseDetectorOut::_process_document_create_tasks(const MSG_PsgDocument &document,
                                                     PoseDetectorOut::Map_Document_Waiting *document_waiting_map_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_process_document_create_tasks(): create tasks for document %ld", document.frame.frame_num);

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
    // _send_document_to_downstreams();
}

void PoseDetectorOut::_connect_to_downstreams()
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
            //         std::bind(&PoseDetectorOut::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&PoseDetectorOut::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&PoseDetectorOut::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_downstreams[it.first] = ds;
    }
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
        auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);

        // FIXME: add timeout condition
        auto task_response = handle.get();
        if (task_response != nullptr) {
            // accepted
            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
                // successfully sent, record this
                task->goal_handle = task_response;
                // task->status = DSTask_PSGDocument::TASK_SENT;
                {
                    auto lock_ptr_document_task_doing = m_impl->sync_document_doing_map.synchronize();
                    (**lock_ptr_document_task_doing)[task->goal_handle] = task;
                }
                tasks_to_remove.push_back(it.first);
            }

            // succeed
            else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                // task->status = DSTask_PSGDocument::TASK_DONE;
                m_psgdoc_task_done.push_back(task);
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
    RCLCPP_INFO(m_impl->logger, "_remove_document_from_buffer(): remove document with frame_num %d", frame_number);
    // if frame_number is not in buffer, do nothing
    if (document_buffer_ptr->find(frame_number) != document_buffer_ptr->end()) {
        document_buffer_ptr->erase(frame_number);
        RCLCPP_INFO(m_impl->logger, "_remove_document_from_buffer(): remove document with frame_num %d SUCCESS", frame_number);
    }
}

void PoseDetectorOut::_remove_bodyposes_from_buffer(int frame_number, std::map<int, PoseDetectorOut::MSG_Bodyposes> *bodyposes_buffer_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_remove_bodyposes_from_buffer(): remove bodyposes with frame_num %d", frame_number);
    // if frame_number is not in buffer, do nothing
    if (bodyposes_buffer_ptr->find(frame_number) != bodyposes_buffer_ptr->end()) {
        bodyposes_buffer_ptr->erase(frame_number);
        RCLCPP_INFO(m_impl->logger, "_remove_bodyposes_from_buffer(): remove bodyposes with frame_num %d SUCCESS", frame_number);
    }
}

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
            //     RCLCPP_INFO(m_impl->logger, "_merge_bodyposes_and_documents(): bodyposes_buffer frame_num %d", it.first);
            // }

            if ((*lock_ptr_bodyposes_buffer)->find(frame_num) != (*lock_ptr_bodyposes_buffer)->end()) {
                bodyposes = (**lock_ptr_bodyposes_buffer)[frame_num];
                has_bodyposes = true;
            }
        }

        if (!has_bodyposes) {
            continue;
        }

        RCLCPP_INFO(m_impl->logger, "_merge_bodyposes_and_documents(): for frame %d", frame_num);
        RCLCPP_INFO(m_impl->logger, "_merge_bodyposes_and_documents(): _merge framenum %ld document and bodyposes", document.frame.frame_num);

        // merge bodyposes and document's persons
        bool is_merged = false;
        for (auto &bodypose : bodyposes) {
            // find the corresponding persons
            auto &persons = document.persons;
            for (auto &person : persons.persons) {
                if (bodypose.uuid == person.uuid) {
                    // merge bodypose and person
                    person.pose = bodypose;
                    is_merged = true;
                    // RCLCPP_INFO(m_impl->logger, "_merge_bodyposes_and_documents(): merged bodypose and person with uuid %s", uuid_to_string(person.uuid.uuid).c_str());
                    break;
                }
            }
        }

        if (is_merged) {
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