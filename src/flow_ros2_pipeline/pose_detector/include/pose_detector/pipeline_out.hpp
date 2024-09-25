#pragma once

#include <map>
#include <memory>
#include <pose_detector/pipeline_out_types.hpp>
#include <psg_actions/action/process_body_poses.hpp>
#include <psg_actions/action/process_psg_document.hpp>
#include <psg_common/psg_common.hpp>
#include <psg_services/srv/status_query.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>


namespace FlowRos2Pipeline
{
class PoseDetectorOutImpl;

class PoseDetectorOut : public rclcpp::Node, public IStartStopProtocol
{
  public:
    class Downstream;
    class DSTask_PsgDocument;

  public:
    using ACT_AcceptBodyposes = psg_actions::action::ProcessBodyPoses;
    using ACT_AcceptDocument = psg_actions::action::ProcessPsgDocument;

    using InitConfig = PoseDetectorOutInitConfig;
    using RuntimeConfig = PoseDetectorOutRuntimeConfig;
    using MSG_PsgDocument = psg_private_msgs::msg::PsgDocument;
    using MSG_Bodypose = psg_public_msgs::msg::BodyPose;
    using MSG_Bodyposes = ACT_AcceptBodyposes::Goal::_body_poses_type;

    using GoalHandle_PsgDocument = rclcpp_action::ClientGoalHandle<ACT_AcceptDocument>::SharedPtr;

    using Map_Document_Waiting = std::map<std::tuple<Downstream *, int>, std::shared_ptr<DSTask_PsgDocument>>;
    using Map_Document_Doing = std::map<GoalHandle_PsgDocument, std::shared_ptr<DSTask_PsgDocument>>;
    using Vec_Document_Done = std::vector<std::shared_ptr<DSTask_PsgDocument>>;

    class Downstream
    {
      public:
        virtual ~Downstream()
        {
        }
        // client to call query service
        rclcpp_action::Client<ACT_AcceptDocument>::SharedPtr accept_document;
        rclcpp_action::Client<ACT_AcceptDocument>::SendGoalOptions accept_document_options;
    };

    class DSTask_PsgDocument
    {
      public:
        MSG_PsgDocument document; // frame associated with this task
        std::shared_ptr<Downstream> downstream;
        GoalHandle_PsgDocument goal_handle; // downstream goal handle
        int retry_times = 0;                // retry times already
    };

  public:
    explicit PoseDetectorOut();

    // initialize with configurations, must be called once before open()
    virtual int init(const std::shared_ptr<InitConfig> &config, const std::shared_ptr<RuntimeConfig> &runtime_config);

    // you can set configuration before open() or after close()
    virtual const std::shared_ptr<InitConfig> &get_init_config() const;

    // modify runtime settings, must be called before start(), after stop() or close()
    virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig> &config);
    virtual const std::shared_ptr<RuntimeConfig> &get_runtime_config() const;

    // can modify runtime config

    // call this after ready() and before you spin this node
    // after calling this, you cannot modify runtime config
    virtual int start() override;

    // cannot modify any config, can call set_xxx() to modify relevant states

    // call this before you modify runtime config
    virtual int stop() override;

    // can modify runtime config

    // get the status code of this node
    virtual int get_status_code() const;

  protected:
    // accept documents from upstream
    rclcpp_action::Server<ACT_AcceptDocument>::SharedPtr m_act_accept_document;
    virtual rclcpp_action::GoalResponse _accept_document_goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ACT_AcceptDocument::Goal> goal);
    virtual rclcpp_action::CancelResponse _accept_document_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle);
    virtual void _accept_document_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle);

    // create tasks
    virtual void _process_document_create_tasks(const MSG_PsgDocument &document, Map_Document_Waiting *document_waiting_map_ptr);


  protected:
    // accept detections from upstream
    rclcpp_action::Server<ACT_AcceptBodyposes>::SharedPtr m_act_accept_detections;
    virtual rclcpp_action::GoalResponse _accept_bodyposes_goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ACT_AcceptBodyposes::Goal> goal);
    virtual rclcpp_action::CancelResponse _accept_bodyposes_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptBodyposes>> goal_handle);
    virtual void _accept_bodyposes_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptBodyposes>> goal_handle);


  protected:
    virtual void _step();

    // find and connect to downstreams
    virtual void _connect_to_downstreams();

    // ping downstream to check if it is alive
    virtual bool _ping(const std::shared_ptr<Downstream> &ds);

    // send document to all pipeline downstreams
    virtual void _send_document_to_downstreams();

    virtual void _declare_all_parameters();

    virtual void _add_document_to_buffer(const MSG_PsgDocument &document,
                                         std::map<int, MSG_PsgDocument> *document_buffer_ptr);
    virtual void _add_bodyposes_to_buffer(const MSG_Bodyposes &bodyposes, const int frame_number,
                                          std::map<int, MSG_Bodyposes> *bodyposes_buffer_ptr);

    virtual void _remove_document_from_buffer(int frame_number, std::map<int, MSG_PsgDocument> *document_buffer_ptr);

    void _remove_bodyposes_from_buffer(int frame_number, std::map<int, MSG_Bodyposes> *bodyposes_buffer_ptr);

    virtual void _merge_bodyposes_and_documents();

  protected:
    // member of pipeline downstreams
    std::map<std::string, std::shared_ptr<Downstream>> m_downstreams;

    // action to be called by upstreams
    rclcpp_action::Server<ACT_AcceptDocument>::SharedPtr m_act_process_document;
    rclcpp_action::Server<ACT_AcceptBodyposes>::SharedPtr m_act_process_bodyposes;

    // configuration
    std::shared_ptr<InitConfig> m_init_config;
    std::shared_ptr<RuntimeConfig> m_runtime_config;

    // impl data
    std::shared_ptr<PoseDetectorOutImpl> m_impl;

    // on-going tasks of psg document processing
    // indexed by (downstream, frame_number)
    Map_Document_Waiting m_psgdoc_task_waiting;
    Map_Document_Doing m_psgdoc_task_doing;
    Vec_Document_Done m_psgdoc_task_done;

    // status code
    int m_status_code = NodeStatusCode::BEFORE_INIT;

    // buffer
    std::map<int, MSG_PsgDocument> m_document_buffer; // indexed by frame number
    std::map<int, MSG_Bodyposes> m_bodyposes_buffer;  // indexed by frame number
};
} // namespace FlowRos2Pipeline