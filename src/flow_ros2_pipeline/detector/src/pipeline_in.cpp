#include <set>
#include <boost/uuid/uuid_generators.hpp>

#include <rcpputils/asserts.hpp>
#include <detector/pipeline_in.hpp>
#include <detector/_pipeline_in.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline {
    DetectorIn::DetectorIn() : Node("detector_in_node")
    {
        m_impl = std::make_shared<DetectorInImpl>(this);

        _declare_all_parameters();

        RCLCPP_INFO(m_impl->logger, "constraction success!");
    }

    int DetectorIn::init(const std::shared_ptr<InitConfig>& config,
                            const std::shared_ptr<RuntimeConfig>& runtime_config) {
        ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::STOPPED,
            "init FAILED! status code is not BEFORE_INIT or STOPPED");

        m_init_config = config;
        m_runtime_config = runtime_config;

        // create server
        m_act_process_document = rclcpp_action::create_server<ACT_AcceptDocument>(
            this, m_init_config->process_document_action,
            std::bind(&DetectorIn::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DetectorIn::_accept_document_cancel_callback, this, std::placeholders::_1),
            std::bind(&DetectorIn::_accept_document_accepted_callback, this, std::placeholders::_1));

        // setup downstreams
        _connect_to_downstreams();

        RCLCPP_INFO(m_impl->logger,
             "m_status_code from %d to %d!",
              m_status_code, NodeStatusCode::INITIALIZED);
        m_status_code = NodeStatusCode::INITIALIZED;
        return ReturnCode::SUCCESS;
    }

    const std::shared_ptr<DetectorIn::InitConfig>& DetectorIn::get_init_config() const {
        return m_init_config;
    }

    int DetectorIn::update_runtime_config(const std::shared_ptr<RuntimeConfig>& config) {
        ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                    m_status_code != NodeStatusCode::BEFORE_INIT,
                "cannot update_runtime_config");

        m_runtime_config = config;
        return ReturnCode::SUCCESS;
    }

    const std::shared_ptr<DetectorIn::RuntimeConfig>& DetectorIn::get_runtime_config() const {
        return m_runtime_config;
    }


    int DetectorIn::start() {
        // the node must be opened
        ROS_ASSERT(m_status_code == NodeStatusCode::INITIALIZED,
                "cannot start because status code is not INITIALIZED");

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

    int DetectorIn::stop() {
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


    int DetectorIn::get_status_code() const {
        return m_status_code;
    }


    rclcpp_action::GoalResponse DetectorIn::_accept_document_goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ACT_AcceptDocument::Goal> goal) {
        RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
        (void)uuid;  // not used
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse DetectorIn::_accept_document_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle) {
        RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
        (void)goal_handle;  // not used
        return rclcpp_action::CancelResponse::REJECT;
    }

    void DetectorIn::_accept_document_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle) {

        const auto& goal = goal_handle->get_goal();

        // FIXME: cache the document, copy it for modify it
        auto document = goal->document;
        const auto& frame = document.frame;

        //add detections_uuid to document
        boost::uuids::uuid uuid = boost::uuids::random_generator()();
        std::copy(uuid.begin(), uuid.end(), document.detections_uuid.uuid.begin());

        //add to memory registry
        _add_document_to_buffer(document);

        RCLCPP_INFO(m_impl->logger, "Accepted document %ld and add it to buffer", document.frame.frame_num);

        //create tasks for all downstreams
        _process_document_create_tasks(document);
        _process_frame_create_tasks(frame);

        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Document accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
    }


    void DetectorIn::_process_document_create_tasks(const MSG_PsgDocument& document){
        //create tasks of this frame for all downstreams
        for(auto& x : m_pipeline_downstreams) {
            auto task = std::make_shared<DSTask_PsgDocument>();
            task->downstream = x.second;
            task->document = document;
            m_psgdoc_task_waiting[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
        }
    }

    void DetectorIn::_process_frame_create_tasks(const MSG_Frame& frame){
        //create tasks of this frame for all downstreams
        for(auto& x : m_model_downstreams) {
            auto task = std::make_shared<DSTask_Frame>();
            task->downstream = x.second;
            task->frame = frame;
            task->detections_uuid = m_document_buffer[frame.frame_num].detections_uuid;
            m_frame_task_waiting[std::make_tuple(task->downstream.get(), frame.frame_num)] = task;
        }
    }


    void DetectorIn::_step() {
        _send_frame_to_downstreams();
        _send_document_to_downstreams();
    }

    void DetectorIn::_connect_to_downstreams() {
        ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

        m_pipeline_downstreams.clear();
        m_model_downstreams.clear();

        for(auto it: m_init_config->pipeline_downstreams){
            auto ds = std::make_shared<DownstreamPipeline>();
            RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

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
                RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
                client->wait_for_action_server();
                RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
            }

            m_pipeline_downstreams[it.first] = ds;
        }

        for(auto it: m_init_config->model_downstreams){
            auto ds = std::make_shared<DownstreamModel>();
            RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

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
                RCLCPP_INFO(m_impl->logger, "waiting for model action server %s", name.c_str());
                client->wait_for_action_server();
                RCLCPP_INFO(m_impl->logger, "model action server %s is ready", name.c_str());
            }

            m_model_downstreams[it.first] = ds;
        }
    }

    void DetectorIn::_send_frame_to_downstreams() {
        if(!m_frame_task_waiting.empty())
        {
            // initiate all waiting tasks
            std::vector<decltype(m_frame_task_waiting)::key_type> tasks_to_remove;

            for(auto& it : m_frame_task_waiting){
                auto& task = it.second;
                ACT_AcceptFrame::Goal goal;
                goal.frame = task->frame;
                // add detections_uuid to goal
                goal.detections_uuid = task->detections_uuid;

                auto ds = task->downstream;
                auto handle = task->downstream->accept_frame->async_send_goal(goal, ds->accept_frame_options);

                // FIXME: add timeout condition
                auto task_response = handle.get();
                if(task_response != nullptr){
                    // accepted
                    if(task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED){
                        //successfully sent, record this
                        task->goal_handle = task_response;
                        // task->status = DSTask_PSGDocument::TASK_SENT;
                        m_frame_task_doing[task->goal_handle] = task;
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
                m_frame_task_waiting.erase(it);
            }
        }

        //for on-going tasks, if it is done, remove it
        if(!m_frame_task_doing.empty()){
            std::vector<GoalHandle_Frame> tasks_to_remove;
            for(auto& it : m_frame_task_doing){
                auto& task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for(auto& it : tasks_to_remove){
                m_frame_task_doing.erase(it);
            }
        }
    }

    void DetectorIn::_send_document_to_downstreams() {
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


    void DetectorIn::_declare_all_parameters() {
        this->declare_parameter<std::string>("process_document_action", "");
        this->declare_parameter<double>("step_interval_ms", -1);
        this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
    }


    void DetectorIn::_add_document_to_buffer(const MSG_PsgDocument& document) {
        m_document_buffer[document.frame.frame_num] = document;
    }

    void DetectorIn::_remove_document_from_buffer(int frame_number) {
        // if frame_number is not in buffer, do nothing
        if(m_document_buffer.find(frame_number) != m_document_buffer.end()){
            m_document_buffer.erase(frame_number);
        }
    }
}