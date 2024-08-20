#include <boost/thread/synchronized_value.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <rclcpp/utilities.hpp>

#include <pose_detector/_pipeline_in.hpp>
#include <pose_detector/pipeline_in.hpp>
#include <rcpputils/asserts.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
PoseDetectorIn::PoseDetectorIn()
    : Node("pose_detector_in_node")
{
    m_impl = std::make_shared<PoseDetectorInImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    m_impl->sync_document_doing_map = &m_psgdoc_task_doing;
    m_impl->sync_detections_waiting_map = &m_detections_task_waiting;
    m_impl->sync_detections_doing_map = &m_detections_task_doing;

    RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int PoseDetectorIn::init(const std::shared_ptr<InitConfig> &config,
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
        std::bind(&PoseDetectorIn::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PoseDetectorIn::_accept_document_cancel_callback, this, std::placeholders::_1),
        std::bind(&PoseDetectorIn::_accept_document_accepted_callback, this, std::placeholders::_1));

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PoseDetectorIn::InitConfig> &PoseDetectorIn::get_init_config() const
{
    return m_init_config;
}

int PoseDetectorIn::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PoseDetectorIn::RuntimeConfig> &PoseDetectorIn::get_runtime_config() const
{
    return m_runtime_config;
}


int PoseDetectorIn::start()
{
    // the node must be opened
    ROS_ASSERT(m_status_code == NodeStatusCode::INITIALIZED,
               "cannot start because status code is not INITIALIZED");

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STARTED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);

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

int PoseDetectorIn::stop()
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
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);
    return ReturnCode::SUCCESS;
}


int PoseDetectorIn::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse PoseDetectorIn::_accept_document_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDocument::Goal> goal)
{
    RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PoseDetectorIn::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PoseDetectorIn::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the document, copy it for modify it
    auto document = goal->document;
    auto &detections = document.detections;

    // generate uuid for every detection and Person
    for (size_t i = 0; i < detections.detections.size(); i++) {
        auto &detection = detections.detections[i];
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        std::copy(uuid.begin(), uuid.end(), detection.uuid.uuid.begin());

        auto &person = document.persons.persons[i];
        std::copy(uuid.begin(), uuid.end(), person.uuid.uuid.begin());
    }

    // create tasks for all downstreams
    {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        _process_document_create_tasks(document, *lock_ptr_psgdoc_task_waiting);
    }
    {
        auto lock_ptr_detections_task_waiting = m_impl->sync_detections_waiting_map.synchronize();
        _process_detections_create_tasks(detections, *lock_ptr_detections_task_waiting);
    }

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}


void PoseDetectorIn::_process_document_create_tasks(const MSG_PsgDocument &document, Map_Document_Waiting *psgdoc_task_waiting_ptr)
{
    // create tasks of this frame for all downstreams
    for (auto &x : m_pipeline_downstreams) {

        auto task = std::make_shared<DSTask_PsgDocument>();
        task->downstream = x.second;
        task->document = document;
        (*psgdoc_task_waiting_ptr)[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
    }
}

void PoseDetectorIn::_process_detections_create_tasks(const MSG_Detections &detections, Map_Detections_Waiting *detections_waiting_map_ptr)
{
    RCLCPP_INFO(m_impl->logger, "create frame %ld detections tasks for downstreams", detections.frame.frame_num);

    // create tasks of this frame for all downstreams
    for (auto &x : m_model_downstreams) {
        auto task = std::make_shared<DSTask_Detections>();
        task->downstream = x.second;
        task->detections = detections;

        (*detections_waiting_map_ptr)[std::make_tuple(task->downstream.get(), detections.frame.frame_num)] = task;
    }
}


void PoseDetectorIn::_step()
{
    _send_detections_to_downstreams();
    _send_document_to_downstreams();
}

void PoseDetectorIn::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

    m_pipeline_downstreams.clear();
    m_model_downstreams.clear();

    for (auto it : m_init_config->pipeline_downstreams) {
        auto ds = std::make_shared<DownstreamPipeline>();
        RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

        // 创建pipeline downstream
        {
            std::string name = it.second.accept_document_action;
            auto client = rclcpp_action::create_client<ACT_AcceptDocument>(this, name);

            ds->accept_document = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&PoseDetectorIn::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&PoseDetectorIn::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&PoseDetectorIn::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_pipeline_downstreams[it.first] = ds;
    }

    for (auto it : m_init_config->model_downstreams) {
        auto ds = std::make_shared<DownstreamModel>();
        RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

        // 创建pipeline downstream
        {
            std::string name = it.second.accept_detections_action;
            auto client = rclcpp_action::create_client<ACT_AcceptDetections>(this, name);

            ds->accept_detections = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&PoseDetectorIn::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&PoseDetectorIn::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&PoseDetectorIn::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            RCLCPP_INFO(m_impl->logger, "waiting for model action server %s", name.c_str());
            client->wait_for_action_server();
            RCLCPP_INFO(m_impl->logger, "model action server %s is ready", name.c_str());
        }

        m_model_downstreams[it.first] = ds;
    }
}

void PoseDetectorIn::_send_detections_to_downstreams()
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
        ACT_AcceptDetections::Goal goal;
        goal.detections = task->detections;

        auto ds = task->downstream;
        auto handle = task->downstream->accept_detections->async_send_goal(goal, ds->accept_detections_options);

        RCLCPP_INFO(m_impl->logger, "[Request Send]framenum: %ld detections", goal.detections.frame.frame_num);

        // FIXME: add timeout condition
        auto task_response = handle.get();
        RCLCPP_INFO(m_impl->logger, "_step frame async_send_goal: %ld SUCCESS", task->detections.frame.frame_num);
        if (task_response != nullptr) {
            // accepted
            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
                // successfully sent, record this
                task->goal_handle = task_response;
                {
                    auto lock_ptr_frame_task_doing = m_impl->sync_detections_doing_map.synchronize();
                    (**lock_ptr_frame_task_doing)[task->goal_handle] = task;
                }
                tasks_to_remove.push_back(it.first);
            }

            // succeed
            else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                m_detections_task_done.push_back(task);
                tasks_to_remove.push_back(it.first);
            }
        }

        // FIXME: what if failed to send many times?
        // you need to terminate a frame, remove it from memory registry
    }

    // remove all sent tasks
    {
        auto lock_ptr_detections_task_waiting = m_impl->sync_detections_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_detections_task_waiting)->erase(it);
        }
    }

    {
        auto lock_ptr_frame_task_doing = m_impl->sync_detections_doing_map.synchronize();
        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_frame_task_doing)->empty()) {
            std::vector<GoalHandle_Detections> tasks_to_remove;
            for (auto &it : (**lock_ptr_frame_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_detections_task_done.push_back(it.second);
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
    m_detections_task_done.clear();
}

void PoseDetectorIn::_send_document_to_downstreams()
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
        ACT_AcceptDocument::Goal goal;
        goal.document = task->document;
        auto ds = task->downstream;
        auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);
        RCLCPP_INFO(m_impl->logger, "_step document async_send_goal: %ld", task->document.frame.frame_num);

        // FIXME: add timeout condition
        auto task_response = handle.get();
        RCLCPP_INFO(m_impl->logger, "_step document async_send_goal: %ld SUCCESS", task->document.frame.frame_num);
        if (task_response != nullptr) {
            // accepted
            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
                // successfully sent, record this
                task->goal_handle = task_response;
                {
                    auto lock_ptr_psgdoc_task_doing = m_impl->sync_document_doing_map.synchronize();
                    (**lock_ptr_psgdoc_task_doing)[task->goal_handle] = task;
                }
                tasks_to_remove.push_back(it.first);
            }

            // succeed
            else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
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


void PoseDetectorIn::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
}
} // namespace FlowRos2Pipeline