#include "psg_common/psg_common.hpp"
#include <functional>
#include <memory>
#include <chrono>
#include <rclcpp/logging.hpp>
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
        this->declare_parameter<std::string>("process_frame_action", "");
        this->declare_parameter<std::string>("downstream_action", "");
        this->declare_parameter<std::string>("status_query_service", "");
        this->declare_parameter<std::string>("send_frame_action", "");
        this->declare_parameter<double>("step_interval_ms", -1);
        this->declare_parameter<double>("timeout_ms_send_frame_to_downstream", -1);
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
                auto client = rclcpp_action::create_client<ACT_AcceptDocument>(this, name);

                ds->handler = client;
                ds->options.goal_response_callback =
                        std::bind(&MasterNode::_process_document_goal_response_callback, this, std::placeholders::_1);
                ds->options.feedback_callback =
                        std::bind(&MasterNode::_process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
                ds->options.result_callback =
                        std::bind(&MasterNode::_process_document_result_callback, this, std::placeholders::_1);

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
        ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT || m_status_code == NodeStatusCode::STOPPED,
            "[MasterNode] init FAILED! status code is not BEFORE_INIT or STOPPED");

        m_init_config = config;
        m_runtime_config = runtime_config;

        m_memory_registry->connect_to_v6d("/var/run/vineyard.sock");

        // create status_query server
        m_srv_status_query = this->create_service<SRV_StatusQuery>(
            m_init_config->status_query_service, std::bind(&MasterNode::_status_query_callback, this, std::placeholders::_1, std::placeholders::_2));


        // create process_frame server
        m_act_accept_frame = rclcpp_action::create_server<ACT_AcceptFrame>(
            this, m_init_config->process_frame_action,
            std::bind(&MasterNode::_accept_frame_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MasterNode::_accept_frame_cancel_callback, this, std::placeholders::_1),
            std::bind(&MasterNode::_accept_frame_accepted_callback, this, std::placeholders::_1));

        // setup downstreams
        _connect_to_downstreams();

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
        ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                    m_status_code != NodeStatusCode::BEFORE_INIT,
                "cannot update_runtime_config");

        m_runtime_config = config;
        return ReturnCode::SUCCESS;
    }

    int MasterNode::get_status_code() const {
        return m_status_code;
    }

    void MasterNode::_process_document_goal_response_callback(
            const rclcpp_action::ClientGoalHandle<MasterNode::ACT_AcceptDocument>::SharedPtr & goal_handle) {
        if(goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
            //TODO: here
        } else {
            RCLCPP_INFO(m_impl->logger, "[MasterNode] Frame rejected");
        }
    }

    void MasterNode::_process_document_feedback_callback(rclcpp_action::ClientGoalHandle<MasterNode::ACT_AcceptDocument>::SharedPtr,
        const std::shared_ptr<const MasterNode::ACT_AcceptDocument::Feedback> feedback) {
        (void)feedback;
    }

    void MasterNode::_process_document_result_callback(
        const rclcpp_action::ClientGoalHandle<MasterNode::ACT_AcceptDocument>::WrappedResult & result) {
        (void)result;

    }


    void MasterNode::_status_query_callback(const std::shared_ptr<SRV_StatusQuery::Request> request,
            std::shared_ptr<SRV_StatusQuery::Response> response) {
        (void)request;  // not used
        RCLCPP_INFO(m_impl->logger, "Received status query request");
        response->status = ReturnCode::SUCCESS;
    }

    rclcpp_action::GoalResponse MasterNode::_accept_frame_goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ACT_AcceptFrame::Goal> goal) {
        RCLCPP_INFO(m_impl->logger, "Received goal request with frame %ld", goal->frame.frame_num);
        (void)uuid;  // not used
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse MasterNode::_accept_frame_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle) {
        RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
        (void)goal_handle;  // not used
        return rclcpp_action::CancelResponse::REJECT;
    }

    void MasterNode::_accept_frame_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle) {

        const auto& goal = goal_handle->get_goal();

        //cache the frame
        const auto& frame = goal->frame;

        //add to memory registry
        _add_frame_to_buffer(frame);

        RCLCPP_INFO(m_impl->logger, "Accepted frame %ld and add it to buffer", frame.cache.id_int);

        //create tasks for all downstreams
        _process_document_create_tasks(frame);

        auto result = std::make_shared<ACT_AcceptFrame::Result>();
        result->return_msg = "Frame accepted";
        result->return_code = ReturnCode::SUCCESS;
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

    void MasterNode::_process_document_create_tasks(const MSG_Frame& frame){
        //create tasks of this frame for all downstreams
        for(auto& x : m_downstreams) {
            auto task = std::make_shared<DSTask_PSGDocument>();
            task->downstream = x.second;
            task->frame = frame;
            m_psgdoc_task_waiting[std::make_tuple(task->downstream.get(), frame.frame_num)] = task;
        }
    }

    int MasterNode::start() {
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

    int MasterNode::stop() {
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

    void MasterNode::_step() {
        std::set<int> useful_frames;
        std::vector<int> useless_frames;


        if(!m_psgdoc_task_waiting.empty())
        {
            // initiate all waiting tasks
            std::vector<decltype(m_psgdoc_task_waiting)::key_type> tasks_to_remove;

            for(auto& it : m_psgdoc_task_waiting){
                auto& task = it.second;
                ACT_AcceptDocument::Goal goal;
                goal.document.frame = task->frame;
                auto ds = task->downstream;
                auto handle = task->downstream->handler->async_send_goal(goal, ds->options);

                RCLCPP_INFO(m_impl->logger, "_step async_send_goal: %ld", task->frame.frame_num);

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
            std::vector<GoalHandle> tasks_to_remove;
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