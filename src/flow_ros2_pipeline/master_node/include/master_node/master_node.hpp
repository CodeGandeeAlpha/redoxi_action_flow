#pragma once
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/types.hpp>

#include <psg_actions/action/process_frame.hpp>
#include <psg_actions/action/process_psg_document.hpp>
#include <psg_common/psg_common.hpp>
#include <psg_private_msgs/msg/psg_document.hpp>
#include <psg_services/srv/status_query.hpp>

#include <master_node/master_node_types.hpp>
#include <master_node/memory_registry.hpp>

namespace FlowRos2Pipeline
{
class MasterNodeImpl;

class MasterNode : public rclcpp::Node, public IStartStopProtocol
{
  public:
    class Downstream;
    class DSTask_PSGDocument;

  public:
    using InitConfig = MasterNodeInitConfig;
    using RuntimeConfig = MasterNodeRuntimeConfig;
    using MSG_Frame = psg_public_msgs::msg::Frame;
    using ACT_AcceptFrame = psg_actions::action::ProcessFrame;
    using ACT_AcceptDocument = psg_actions::action::ProcessPsgDocument;
    using SRV_StatusQuery = psg_services::srv::StatusQuery;

    using GoalHandle = rclcpp_action::ClientGoalHandle<ACT_AcceptDocument>::SharedPtr;

    using Map_Document_Waiting = std::map<std::tuple<Downstream *, int>, std::shared_ptr<DSTask_PSGDocument>>;
    using Map_Document_Doing = std::map<GoalHandle, std::shared_ptr<DSTask_PSGDocument>>;
    using Vec_Document_Done = std::vector<std::shared_ptr<DSTask_PSGDocument>>;
    class Downstream
    {
      public:
        virtual ~Downstream()
        {
        }
        // client to call query service
        rclcpp_action::Client<ACT_AcceptDocument>::SharedPtr handler;
        rclcpp_action::Client<ACT_AcceptDocument>::SendGoalOptions options;
    };

    class DSTask_PSGDocument
    {
      public:
        MSG_Frame frame; // frame associated with this task
        std::shared_ptr<Downstream> downstream;
        GoalHandle goal_handle; // downstream goal handle
        int retry_times = 0;    // retry times already
    };

  public:
    MasterNode();
    virtual ~MasterNode()
    {
    }

    // initialize with configurations, must be called once before open()
    virtual int init(const std::shared_ptr<InitConfig> &config, const std::shared_ptr<RuntimeConfig> &runtime_config);

    // you can set configuration before starting this node
    virtual const std::shared_ptr<InitConfig> &get_init_config() const;

    // modify runtime settings, must be called after open() or stop()
    virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig> &config);
    virtual const std::shared_ptr<RuntimeConfig> &get_runtime_config() const;

    // get the status code of this node
    virtual int get_status_code() const;

    // implement start/stop protocol
    virtual int start() override;
    virtual int stop() override;

  protected:
    // service to be called by upstreams

    // query service
    rclcpp::Service<SRV_StatusQuery>::SharedPtr m_srv_status_query;
    virtual void _status_query_callback(const std::shared_ptr<SRV_StatusQuery::Request> request,
                                        std::shared_ptr<SRV_StatusQuery::Response> response);

    // accept frames from upstream
    rclcpp_action::Server<ACT_AcceptFrame>::SharedPtr m_act_accept_frame;
    virtual rclcpp_action::GoalResponse _accept_frame_goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ACT_AcceptFrame::Goal> goal);
    virtual rclcpp_action::CancelResponse _accept_frame_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle);
    virtual void _accept_frame_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptFrame>> goal_handle);

  protected:
    // downstream action handlers

    // psg document downstreams
    std::map<std::string, std::shared_ptr<Downstream>> m_downstreams;
    // virtual void process_document_send_goals();
    virtual void _process_document_goal_response_callback(
        const rclcpp_action::ClientGoalHandle<ACT_AcceptDocument>::SharedPtr &goal_handle);
    virtual void _process_document_feedback_callback(rclcpp_action::ClientGoalHandle<ACT_AcceptDocument>::SharedPtr,
                                                     const std::shared_ptr<const ACT_AcceptDocument::Feedback> feedback);
    virtual void _process_document_result_callback(
        const rclcpp_action::ClientGoalHandle<ACT_AcceptDocument>::WrappedResult &result);
    virtual void _process_document_create_tasks(const MSG_Frame &frame, Map_Document_Waiting *document_waiting_map_ptr);

  protected:
    virtual void _declare_all_parameters();
    virtual void _step(); // called by timer periodically
    virtual void _add_frame_to_buffer(const MSG_Frame &frame, std::map<int, MasterNode::MSG_Frame> *frame_buffer_ptr);
    virtual void _remove_frame_from_buffer(int frame_number, std::map<int, MSG_Frame> *frame_buffer_ptr, bool remove_memory_entry);
    // find and connect to downstreams
    virtual void _connect_to_downstreams();
    // check if all downstreams are ready to accept new frame
    virtual bool _ping(std::shared_ptr<Downstream> ds);

    // buffer
    std::map<int, MSG_Frame> m_frame_buffer; // indexed by frame number

    // on-going tasks of psg document processing
    // indexed by (downstream, frame_number)
    Map_Document_Waiting m_psgdoc_task_waiting;
    Map_Document_Doing m_psgdoc_task_doing;
    Vec_Document_Done m_psgdoc_task_done;

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

} // namespace FlowRos2Pipeline