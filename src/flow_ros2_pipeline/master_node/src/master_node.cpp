#include <functional>
#include <memory>
#include <chrono>
#include <set>
#include <stdexcept>

#include <rclcpp_action/create_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rcpputils/asserts.hpp>

#include <master_node/master_node.hpp>
#include <master_node/_master_node.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

#define COMPILE_MASTER_NODE
#ifdef COMPILE_MASTER_NODE
namespace FlowRos2Pipeline {
    MasterNode::MasterNode() : Node("master_node")
    {
        m_impl = std::make_shared<MasterNodeImpl>(this);

        _declare_all_parameters();

        m_memory_registry = std::make_shared<MemoryRegistry>(this);

        RCLCPP_INFO(m_impl->logger, "[MasterNode] constraction success!");
    }

    void MasterNode::_declare_all_parameters() {
        this->declare_parameter<std::string>("downstream_action", "");
        this->declare_parameter<std::string>("status_query_service", "");
        this->declare_parameter<std::string>("send_frame_action", "");
        this->declare_parameter<double>("frame_internal_ms", -1);
    }

    void MasterNode::_connect_to_downstreams(){
        ROS_ASSERT(m_init_config != nullptr, "[MasterNode] m_init_config is nullptr");

        m_downstreams.clear();
        for(auto it: m_init_config->downstreams){
            auto ds = std::make_shared<Downstream>();
            RCLCPP_INFO(m_impl->logger, "[MasterNode] connecting to downstream %s", it.first.c_str());

            // 创建accept_frame_client
            {
                std::string name = it.second.accept_document_action;
                auto client = rclcpp_action::create_client<DownstreamAcceptDocumentAction>(this, name);

                ds->handler = client;
                ds->options.goal_response_callback =
                        std::bind(&MasterNode::process_document_goal_response_callback, this, std::placeholders::_1);
                ds->options.feedback_callback =
                        std::bind(&MasterNode::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
                ds->options.result_callback =
                        std::bind(&MasterNode::process_document_result_callback, this, std::placeholders::_1);

                // wait until the action server is ready
                RCLCPP_INFO(m_impl->logger, "[MasterNode] waiting for action server %s", name.c_str());
                client->wait_for_action_server();
                RCLCPP_INFO(m_impl->logger, "[MasterNode] action server %s is ready", name.c_str());
            }

            m_downstreams[it.first] = ds;
        }
    }

    int MasterNode::init(const std::shared_ptr<InitConfig>& config,
                            const std::shared_ptr<RuntimeConfig>& runtime_config) {
        if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
            RCLCPP_ERROR(m_impl->logger, "[MasterNode] init FAILED! status code is not BEFORE_INIT or CLOSED");
            return ReturnCode::ERROR;
        }
        ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT,
            "[MasterNode] init FAILED! status code is not BEFORE_INIT");

        m_init_config = config;
        m_runtime_config = runtime_config;

        m_memory_registry->connect_to_v6d("/var/run/vineyard.sock");

        // setup downstreams
        _connect_to_downstreams();

        // create status_query server
        std::string status_query_service = this->get_parameter("status_query_service").as_string();
        // m_srv_status_query = this->create_service<MSG_StatusQuery>(
        //     status_query_service, &MasterNode::status_query_callback);
        m_srv_status_query = this->create_service<MSG_StatusQuery>(
            status_query_service, std::bind(&MasterNode::status_query_callback, this, std::placeholders::_1, std::placeholders::_2));


        // create send_frame server
        std::string send_frame_action = this->get_parameter("send_frame_action").as_string();
        m_act_accept_frame = rclcpp_action::create_server<ACT_AcceptFrame>(
            this, send_frame_action,
            std::bind(&MasterNode::accept_frame_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MasterNode::accept_frame_cancel_callback, this, std::placeholders::_1),
            std::bind(&MasterNode::accept_frame_accepted_callback, this, std::placeholders::_1));

        RCLCPP_INFO(m_impl->logger,
             "[MasterNode] m_status_code from %d to %d!",
              m_status_code, NodeStatusCode::INITIALIZED);
        m_status_code = NodeStatusCode::INITIALIZED;
        return ReturnCode::SUCCESS;
    }

    const std::shared_ptr<MasterNode::InitConfig>& MasterNode::get_init_config() const {
        return m_init_config;
    }

    const std::shared_ptr<MasterNode::RuntimeConfig>& MasterNode::get_runtime_config() const {
        return m_runtime_config;
    }

    int MasterNode::update_runtime_config(const std::shared_ptr<RuntimeConfig>& config) {
        // if (m_status_code != NodeStatusCode::OPENED) {
        //     RCLCPP_ERROR(m_impl->logger, "[MasterNode] update_runtime_config FAILED! status code is not OPENED");
        //     return ReturnCode::ERROR;
        // }

        m_runtime_config = config;
        return ReturnCode::SUCCESS;
    }

    int MasterNode::get_status_code() const {
        return m_status_code;
    }

    void MasterNode::process_document_goal_response_callback(
            const rclcpp_action::ClientGoalHandle<MasterNode::DownstreamAcceptDocumentAction>::SharedPtr & goal_handle) {
        if(goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
            //TODO: here
        } else {
            RCLCPP_INFO(m_impl->logger, "[MasterNode] Frame rejected");
        }
    }

    void MasterNode::process_document_feedback_callback(rclcpp_action::ClientGoalHandle<MasterNode::DownstreamAcceptDocumentAction>::SharedPtr,
        const std::shared_ptr<const MasterNode::DownstreamAcceptDocumentAction::Feedback> feedback) {
        (void)feedback;
    }

    void MasterNode::process_document_result_callback(
        const rclcpp_action::ClientGoalHandle<MasterNode::DownstreamAcceptDocumentAction>::WrappedResult & result) {
        (void)result;

    }


    void MasterNode::status_query_callback(const std::shared_ptr<MSG_StatusQuery::Request> request,
            std::shared_ptr<MSG_StatusQuery::Response> response) {
        (void)request;  // not used
        RCLCPP_INFO(m_impl->logger, "Received status query request");
        response->status = ReturnCode::SUCCESS;
    }

    rclcpp_action::GoalResponse MasterNode::accept_frame_goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ACT_AcceptFrame::Goal> goal) {
        RCLCPP_INFO(m_impl->logger, "Received goal request with frame %ld", goal->frame.frame_num);
        (void)uuid;  // not used
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse MasterNode::accept_frame_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle) {
        RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
        (void)goal_handle;  // not used
        return rclcpp_action::CancelResponse::REJECT;
    }

    void MasterNode::accept_frame_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle) {

        const auto& goal = goal_handle->get_goal();

        //cache the frame
        const auto& frame = goal->frame;

        //add to memory registry
        _add_frame_to_buffer(frame);

        RCLCPP_INFO(m_impl->logger, "Accepted frame %ld and add it to buffer", frame.cache.id_int);

        //create tasks for all downstreams
        // process_document_create_tasks(frame);


        auto result = std::make_shared<ACT_AcceptFrame::Result>();
        goal_handle->succeed(result);
    }

    void MasterNode::_add_frame_to_buffer(const MSG_Frame& frame) {
        m_frame_buffer[frame.frame_num] = frame;

        //add to memory registry
        MemoryEntry e;
        e.frame_number = frame.frame_num;
        e.name = "frame";

        if(!frame.cache.has_int_id)
            throw std::runtime_error("frame.cache has no int_id");

        e.v6d_object_id = frame.cache.id_int;
        m_memory_registry->add_entry(e);
    }

    void MasterNode::_remove_frame_from_buffer(int frame_number, bool remove_memory_entry){
        m_frame_buffer.erase(frame_number);
        if(remove_memory_entry)
            m_memory_registry->remove_entries_by_frame(frame_number);
    }

    void MasterNode::process_document_create_tasks(const MSG_Frame& frame){
        //create tasks of this frame for all downstreams
        for(auto& x : m_downstreams) {
            auto task = std::make_shared<DSTask_PSGDocument>();
            task->downstream = x.second;
            task->frame = frame;
            m_psgdoc_task_waiting[std::make_tuple(task->downstream.get(), frame.frame_num)] = task;
        }
    }

    int MasterNode::start() {
        if (m_status_code != NodeStatusCode::INITIALIZED && m_status_code != NodeStatusCode::STOPPED) {
            RCLCPP_ERROR(m_impl->logger, "[MasterNode] start FAILED! status code is not INITIALIZED or STOPPED");
            return ReturnCode::ERROR;
        }

        //start timer
        // m_impl->timer = this->create_wall_timer(
        //     std::chrono::milliseconds((int)m_runtime_config->frame_internal_ms),
        //     std::bind(&MasterNode::_step, this));

        m_status_code = NodeStatusCode::STARTED;

        m_impl->step_running = true;
        m_impl->step_thread = std::make_shared<std::thread>(
            [this](){
                while(rclcpp::ok() && m_impl->step_running){
                    _step();
                    std::this_thread::sleep_for(std::chrono::milliseconds((int)DefaultNodeStepIntervalMs));
                }
            }
        );

        return ReturnCode::SUCCESS;
    }

    int MasterNode::stop() {
        if (m_status_code != NodeStatusCode::STARTED) {
            RCLCPP_ERROR(m_impl->logger, "[MasterNode] stop FAILED! status code is not STARTED");
            return ReturnCode::ERROR;
        }

        // if(m_impl->timer)
        //     m_impl->timer->cancel();
        // else
        //     throw std::runtime_error("timer is not initialized but stop() is called");

        m_status_code = NodeStatusCode::STOPPED;

        //terminate step thread
        m_impl->step_running = false;
        if(m_impl->step_thread){
            m_impl->step_thread->join();
            m_impl->step_thread = nullptr;
        }

        return ReturnCode::SUCCESS;
    }

    void MasterNode::_step() {
        if(!m_psgdoc_task_waiting.empty())
        {
            // initiate all waiting tasks
            std::vector<decltype(m_psgdoc_task_waiting)::key_type> tasks_to_remove;

            for(auto& it : m_psgdoc_task_waiting){
                auto& task = it.second;
                DownstreamAcceptDocumentAction::Goal goal;
                goal.document.frame = task->frame;
                auto ds = task->downstream;
                auto handle = task->downstream->handler->async_send_goal(goal, ds->options);

                // FIXME: add timeout condition
                auto task_response = handle.get();
                if(task_response != nullptr){
                    // accepted?
                    if(task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED){
                        //successfully sent, record this
                        task->goal_id = task_response->get_goal_id();
                        task->status = DSTask_PSGDocument::TASK_SENT;
                        m_psgdoc_task_doing[task->goal_id] = task;
                        tasks_to_remove.push_back(it.first);
                    }
                }
                else {
                    // rejected
                    task->status = DSTask_PSGDocument::TASK_FAILED;
                }

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
            std::vector<GoalID> tasks_to_remove;
            for(auto& it : m_psgdoc_task_doing){
                auto& task = it.second;
                if(task->status == DSTask_PSGDocument::TASK_DONE || task->status == DSTask_PSGDocument::TASK_FAILED){
                    tasks_to_remove.push_back(it.first);
                }
            }

            for(auto& it : tasks_to_remove){
                m_psgdoc_task_doing.erase(it);
            }
        }

        //if a frame has no waiting tasks or running tasks associated with it, remove it from buffer
        std::set<int> useful_frames;
        std::vector<int> useless_frames;
        for(auto& it : m_psgdoc_task_waiting){
            useful_frames.insert(it.second->frame.frame_num);
        }
        for(auto& it : m_psgdoc_task_doing){
            useful_frames.insert(it.second->frame.frame_num);
        }
        for(auto& it : m_frame_buffer){
            // if not useful, it is useless
            if(useful_frames.find(it.first) == useful_frames.end()){
                useless_frames.push_back(it.first);
            }
        }

        // remove useless frames
        for(auto& it : useless_frames){
            _remove_frame_from_buffer(it, false);
        }
    }
}

#endif