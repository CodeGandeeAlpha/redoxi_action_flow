#include <set>
#include <boost/uuid/uuid_generators.hpp>

#include <rcpputils/asserts.hpp>
#include <detector/pipeline_out.hpp>
#include <detector/_pipeline_out.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline {
    DetectorOut::DetectorOut() : Node("detector_out_node")
    {
        m_impl = std::make_shared<DetectorOutImpl>(this);

        _declare_all_parameters();

        RCLCPP_INFO(m_impl->logger, "constraction success!");
    }

    int DetectorOut::init(const std::shared_ptr<InitConfig>& config,
                            const std::shared_ptr<RuntimeConfig>& runtime_config) {
        if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
            RCLCPP_ERROR(m_impl->logger, "init FAILED! status code is not BEFORE_INIT or CLOSED");
            return ReturnCode::ERROR;
        }
        ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT,
            "init FAILED! status code is not BEFORE_INIT");

        m_init_config = config;
        m_runtime_config = runtime_config;

        // setup downstreams
        _connect_to_downstreams();

        // create process document server
        std::string process_document_action = this->get_parameter(m_init_config->process_document_action).as_string();
        m_act_process_document = rclcpp_action::create_server<ACT_AcceptDocument>(
            this, process_document_action,
            std::bind(&DetectorOut::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DetectorOut::_accept_document_cancel_callback, this, std::placeholders::_1),
            std::bind(&DetectorOut::_accept_document_accepted_callback, this, std::placeholders::_1));

        // create process detections server
        std::string process_detections_action = this->get_parameter(m_init_config->process_detections_action).as_string();
        m_act_process_detections = rclcpp_action::create_server<ACT_AcceptDetections>(
            this, process_detections_action,
            std::bind(&DetectorOut::_accept_detections_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DetectorOut::_accept_detections_cancel_callback, this, std::placeholders::_1),
            std::bind(&DetectorOut::_accept_detections_accepted_callback, this, std::placeholders::_1));

        RCLCPP_INFO(m_impl->logger,
             "m_status_code from %d to %d!",
              m_status_code, NodeStatusCode::INITIALIZED);
        m_status_code = NodeStatusCode::INITIALIZED;
        return ReturnCode::SUCCESS;
    }

    const std::shared_ptr<DetectorOut::InitConfig>& DetectorOut::get_init_config() const {
        return m_init_config;
    }

    int DetectorOut::update_runtime_config(const std::shared_ptr<RuntimeConfig>& config) {
        ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                    m_status_code != NodeStatusCode::BEFORE_INIT,
                "cannot update_runtime_config");

        m_runtime_config = config;
        return ReturnCode::SUCCESS;
    }

    const std::shared_ptr<DetectorOut::RuntimeConfig>& DetectorOut::get_runtime_config() const {
        return m_runtime_config;
    }


    int DetectorOut::start() {
        // the node must be opened
        ROS_ASSERT(m_status_code == NodeStatusCode::OPENED,
                "cannot start because status code is not OPENED");

        RCLCPP_INFO(m_impl->logger,
             "m_status_code from %d to %d!",
              m_status_code, NodeStatusCode::STARTED);

        m_status_code = NodeStatusCode::STARTED;

        m_impl->step_running = true;
        m_impl->step_thread = std::make_shared<std::thread>(
            [this](){
                while(rclcpp::ok() && m_impl->step_running){
                    _step();
                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
                }
            }
        );

        return ReturnCode::SUCCESS;
    }

    int DetectorOut::stop() {
        // only stoppable if the node is started
        ROS_ASSERT(m_status_code == NodeStatusCode::STARTED,
                "cannot stop because status code is not STARTED");

        //terminate step thread
        m_impl->step_running = false;
        if(m_impl->step_thread){
            m_impl->step_thread->join();
            m_impl->step_thread = nullptr;
        }

        RCLCPP_INFO(m_impl->logger,
             "m_status_code from %d to %d!",
              m_status_code, NodeStatusCode::STOPPED);

        m_status_code = NodeStatusCode::STOPPED;
        return ReturnCode::SUCCESS;
    }


    int DetectorOut::get_status_code() const {
        return m_status_code;
    }


    rclcpp_action::GoalResponse DetectorOut::_accept_document_goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ACT_AcceptDocument::Goal> goal) {
        RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
        (void)uuid;  // not used
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse DetectorOut::_accept_document_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle) {
        RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
        (void)goal_handle;  // not used
        return rclcpp_action::CancelResponse::REJECT;
    }

    void DetectorOut::_accept_document_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle) {

        const auto& goal = goal_handle->get_goal();

        //cache the document
        const auto& document = goal->document;

        //add to buffer
        _add_document_to_buffer(document);

        RCLCPP_INFO(m_impl->logger, "Accepted document %ld and add it to buffer", document.frame.frame_num);

        // //create tasks for all downstreams
        // _process_document_create_tasks(document);

        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Document accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
    }

    rclcpp_action::GoalResponse DetectorOut::_accept_detections_goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ACT_AcceptDetections::Goal> goal) {
        RCLCPP_INFO(m_impl->logger, "Received goal request with detections with frame_num %ld", goal->detections.frame.frame_num);
        (void)uuid;  // not used
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse DetectorOut::_accept_detections_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle) {
        RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
        (void)goal_handle;  // not used
        return rclcpp_action::CancelResponse::REJECT;
    }

    void DetectorOut::_accept_detections_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle) {

        const auto& goal = goal_handle->get_goal();

        //cache the detections
        const auto& detections = goal->detections;

        //add to buffer
        _add_detections_to_buffer(detections);

        RCLCPP_INFO(m_impl->logger, "add detections to buffer");

        auto result = std::make_shared<ACT_AcceptDetections::Result>();
        result->return_msg = "Detections accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
    }


    void DetectorOut::_process_document_create_tasks(const MSG_PsgDocument& document){
        //create tasks of this frame for all downstreams
        for(auto& x : m_downstreams) {
            auto task = std::make_shared<DSTask_PsgDocument>();
            task->downstream = x.second;
            task->document = document;
            m_psgdoc_task_waiting[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
        }
    }


    void DetectorOut::_step() {  // TODO: merge detections and documents
        _merge_detections_and_documents();
        _send_document_to_downstreams();
    }

    void DetectorOut::_connect_to_downstreams() {
        ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

        m_downstreams.clear();

        for(auto it: m_init_config->downstreams){
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

    void DetectorOut::_send_document_to_downstreams() {
        std::set<int> useful_documents;
        std::vector<int> useless_documents;

        if(!m_psgdoc_task_waiting.empty())
        {
            // initiate all waiting tasks
            std::vector<decltype(m_psgdoc_task_waiting)::key_type> tasks_to_remove;

            for(auto& it : m_psgdoc_task_waiting){
                auto& task = it.second;
                ACT_AcceptDocument::Goal goal;
                goal.document = task->document;
                auto ds = task->downstream;
                auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);

                // FIXME: add timeout condition
                auto task_response = handle.get();
                if(task_response != nullptr){
                    // accepted
                    if(task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED){
                        //successfully sent, record this
                        task->goal_handle = task_response;
                        // task->status = DSTask_PSGDocument::TASK_SENT;
                        m_psgdoc_task_doing[task->goal_handle] = task;
                        tasks_to_remove.push_back(it.first);
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED){
                        // task->status = DSTask_PSGDocument::TASK_DONE;
                        tasks_to_remove.push_back(it.first);
                    }
                }
                // else {
                //     // rejected
                //     task->status = DSTask_PSGDocument::TASK_FAILED;
                // }

                //FIXME: what if failed to send many times?
                //you need to terminate a frame, remove it from memory registry
            }

            // remove all sent tasks
            for(auto& it : tasks_to_remove){
                m_psgdoc_task_waiting.erase(it);
            }
        }

        //for on-going tasks, if it is done, remove it
        if(!m_psgdoc_task_doing.empty()){
            std::vector<GoalHandle_PsgDocument> tasks_to_remove;
            for(auto& it : m_psgdoc_task_doing){
                auto& task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for(auto& it : tasks_to_remove){
                m_psgdoc_task_doing.erase(it);
            }
        }

        //if a frame has no waiting tasks or running tasks associated with it, remove it from buffer
        for(auto& it : m_psgdoc_task_waiting){
            useful_documents.insert(it.second->document.frame.frame_num);
        }
        for(auto& it : m_psgdoc_task_doing){
            useful_documents.insert(it.second->document.frame.frame_num);
        }
        for(auto& it : m_document_buffer){
            // if not useful, it is useless
            if(useful_documents.find(it.first) == useful_documents.end()){
                useless_documents.push_back(it.first);
            }
        }

        // remove useless documents
        for(auto& it : useless_documents){
            _remove_document_from_buffer(it);
        }
    }


    void DetectorOut::_declare_all_parameters() {
        this->declare_parameter<double>("step_interval_ms", -1);
        this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
    }


    void DetectorOut::_add_document_to_buffer(const MSG_PsgDocument& document) {
        m_document_buffer[document.frame.frame_num] = document;
    }

    void DetectorOut::_add_detections_to_buffer(const MSG_Detections& detections) {
        m_detections_buffer[detections.frame.frame_num] = detections;
    }

    void DetectorOut::_remove_document_from_buffer(int frame_number) {
        // if frame_number is not in buffer, do nothing
        if(m_document_buffer.find(frame_number) != m_document_buffer.end()){
            m_document_buffer.erase(frame_number);
        }
    }

    void DetectorOut::_merge_detections_and_documents() {
        for (auto& it : m_document_buffer) {
            auto& document = it.second;
            auto frame_num = it.first;
            if (m_detections_buffer.find(frame_num) != m_detections_buffer.end()) {
                auto& detections = m_detections_buffer[frame_num];

                if (document.detections_uuid == detections.uuid) {
                    // merge detections and documents
                    document.detections = detections;
                    _process_document_create_tasks(document);

                    // remove detections from buffer
                    m_detections_buffer.erase(frame_num);
                }
            }
        }
    }
}