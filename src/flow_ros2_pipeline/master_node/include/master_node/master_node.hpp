#pragma once
#include <memory>
#include <rclcpp_action/types.hpp>
#include <string>
#include <map>
#include <tuple>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <psg_common/psg_common.hpp>
#include <psg_actions/action/send_frame.hpp>
#include <psg_services/srv/status_query.hpp>
#include <psg_private_msgs/msg/psg_document.hpp>
#include <psg_actions/action/process_psg_document.hpp>

#include <master_node/memory_registry.hpp>
#include <master_node/master_node_types.hpp>

namespace FlowRos2Pipeline {
    class MasterNodeImpl;

    class MasterNode : public rclcpp::Node, public IStartStopProtocol {
    public:
        class DownstreamFunctions{
        public:
            using DocumentAction = psg_actions::action::ProcessPsgDocument;
        };

        class DS_PSGDocument{
        public:
            virtual ~DS_PSGDocument(){}
            // client to call query service
            rclcpp_action::Client<DownstreamFunctions::DocumentAction>::SharedPtr handler;
            rclcpp_action::Client<DownstreamFunctions::DocumentAction>::SendGoalOptions options;
        };

        using InitConfig = MasterNodeInitConfig;
        using RuntimeConfig = MasterNodeRuntimeConfig;

    public:
        MasterNode();
        virtual ~MasterNode() {}

        // initialize with configurations, must be called once before open()
        virtual int init(const std::shared_ptr<InitConfig>& config, const std::shared_ptr<RuntimeConfig>& runtime_config);

        // you can set configuration before starting this node
        virtual const std::shared_ptr<InitConfig>& get_init_config() const;

        // modify runtime settings, must be called after open() or stop()
        virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig> & config);
        virtual const std::shared_ptr<RuntimeConfig>& get_runtime_config() const;

        // get the status code of this node
        virtual int get_status_code() const;

        // implement start/stop protocol
        virtual int start() override;
        virtual int stop() override;

    protected:
        // service to be called by upstreams

        // query service
        using MSG_StatusQuery = psg_services::srv::StatusQuery;
        rclcpp::Service<MSG_StatusQuery>::SharedPtr m_srv_status_query;
        virtual void status_query_callback(const std::shared_ptr<MSG_StatusQuery::Request> request,
            std::shared_ptr<MSG_StatusQuery::Response> response);

        // accept frames from upstream
        using MSG_Frame = psg_public_msgs::msg::Frame;
        using ACT_AcceptFrame = psg_actions::action::SendFrame;
        rclcpp_action::Server<ACT_AcceptFrame>::SharedPtr m_act_accept_frame;
        virtual rclcpp_action::GoalResponse accept_frame_goal_callback(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const ACT_AcceptFrame::Goal> goal);
        virtual rclcpp_action::CancelResponse accept_frame_cancel_callback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle);
        virtual void accept_frame_accepted_callback(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle);

    protected:
        // downstream action handlers

        // psg document downstreams
        std::map<std::string, std::shared_ptr<DS_PSGDocument>> m_ds_psgdocument;
        // virtual void process_document_send_goals();
        virtual void process_document_goal_response_callback(
            const rclcpp_action::ClientGoalHandle<DownstreamFunctions::DocumentAction>::SharedPtr & goal_handle);
        virtual void process_document_feedback_callback(rclcpp_action::ClientGoalHandle<DownstreamFunctions::DocumentAction>::SharedPtr,
            const std::shared_ptr<const DownstreamFunctions::DocumentAction::Feedback> feedback);
        virtual void process_document_result_callback(
            const rclcpp_action::ClientGoalHandle<DownstreamFunctions::DocumentAction>::WrappedResult & result);
        virtual void process_document_create_tasks(const MSG_Frame& frame);

        using GoalID = rclcpp_action::GoalUUID;
        class DSTask_PSGDocument{
        public:
            MSG_Frame frame;  // frame associated with this task
            std::shared_ptr<DS_PSGDocument> downstream;
            GoalID goal_id; //id of the goal already sent to the downstream

            enum TaskStatus{
                TASK_NOT_SENT = 0,
                TASK_SENT = 1,
                TASK_DONE = 2,
                TASK_FAILED = 3,
            };
            TaskStatus status = TASK_NOT_SENT;
        };

    protected:
        virtual void _declare_all_parameters();
        virtual void _step();   //called by timer periodically
        virtual void _add_frame_to_buffer(const MSG_Frame& frame);
        virtual void _remove_frame_from_buffer(int frame_number, bool remove_memory_entry);

        // buffer
        std::map<int, MSG_Frame> m_frame_buffer;    // indexed by frame number

        // on-going tasks of psg document processing
        // indexed by (downstream, frame_number)
        std::map<std::tuple<DS_PSGDocument*, int>, std::shared_ptr<DSTask_PSGDocument>> m_psgdoc_task_waiting;
        std::map<GoalID, std::shared_ptr<DSTask_PSGDocument>> m_psgdoc_task_doing;

        // configuration
        std::shared_ptr<InitConfig> m_init_config;
        std::shared_ptr<RuntimeConfig> m_runtime_config;

        // impl data
        std::shared_ptr<MasterNodeImpl> m_impl;

        // memory registry
        std::shared_ptr<MemoryRegistry> m_memory_registry;

        // status code
        int m_status_code = NodeStatusCode::BEFORE_INIT;
    };

}