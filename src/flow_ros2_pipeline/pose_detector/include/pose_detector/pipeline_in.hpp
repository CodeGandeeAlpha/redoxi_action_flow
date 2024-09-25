#pragma once

#include <memory>
#include <string>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <psg_actions/action/process_detections.hpp>
#include <psg_actions/action/process_psg_document.hpp>
#include <psg_common/psg_common.hpp>
#include <psg_services/srv/status_query.hpp>

#include <pose_detector/pipeline_in_types.hpp>


namespace FlowRos2Pipeline
{
class PoseDetectorInImpl;

class PoseDetectorIn : public rclcpp::Node, public IStartStopProtocol
{
  public:
    class DownstreamPipeline;
    class DownstreamModel;
    class DSTask_PsgDocument;
    class DSTask_Detections;

  public:
    using ACT_AcceptDetections = psg_actions::action::ProcessDetections;
    using ACT_AcceptDocument = psg_actions::action::ProcessPsgDocument;

    using InitConfig = PoseDetectorInInitConfig;
    using RuntimeConfig = PoseDetectorInRuntimeConfig;
    using MSG_Detections = psg_public_msgs::msg::Detections;
    using MSG_PsgDocument = psg_private_msgs::msg::PsgDocument;

    using GoalHandle_PsgDocument = rclcpp_action::ClientGoalHandle<ACT_AcceptDocument>::SharedPtr;
    using GoalHandle_Detections = rclcpp_action::ClientGoalHandle<ACT_AcceptDetections>::SharedPtr;


    using Map_Document_Waiting = std::map<std::tuple<DownstreamPipeline *, int>, std::shared_ptr<DSTask_PsgDocument>>;
    using Map_Document_Doing = std::map<GoalHandle_PsgDocument, std::shared_ptr<DSTask_PsgDocument>>;
    using Vec_Document_Done = std::vector<std::shared_ptr<DSTask_PsgDocument>>;
    using Map_Detections_Waiting = std::map<std::tuple<DownstreamModel *, int>, std::shared_ptr<DSTask_Detections>>;
    using Map_Detections_Doing = std::map<GoalHandle_Detections, std::shared_ptr<DSTask_Detections>>;
    using Vec_Detections_Done = std::vector<std::shared_ptr<DSTask_Detections>>;

    class DownstreamPipeline
    {
      public:
        virtual ~DownstreamPipeline()
        {
        }
        // client to call query service
        rclcpp_action::Client<ACT_AcceptDocument>::SharedPtr accept_document;
        rclcpp_action::Client<ACT_AcceptDocument>::SendGoalOptions accept_document_options;
    };

    class DownstreamModel
    {
      public:
        virtual ~DownstreamModel()
        {
        }
        // client to call query service
        rclcpp_action::Client<ACT_AcceptDetections>::SharedPtr accept_detections;
        rclcpp_action::Client<ACT_AcceptDetections>::SendGoalOptions accept_detections_options;
    };


    class DSTask_Detections
    {
      public:
        MSG_Detections detections; // detections associated with this task
        std::shared_ptr<DownstreamModel> downstream;
        GoalHandle_Detections goal_handle; // downstream goal handle
        int retry_times = 0;               // retry times already
    };

    class DSTask_PsgDocument
    {
      public:
        MSG_PsgDocument document; // document associated with this task
        std::shared_ptr<DownstreamPipeline> downstream;
        GoalHandle_PsgDocument goal_handle; // downstream goal handle
        int retry_times = 0;                // retry times already
    };

  public:
    explicit PoseDetectorIn();

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
    virtual void _process_detections_create_tasks(const MSG_Detections &detections, Map_Detections_Waiting *detections_waiting_map_ptr);

  protected:
    virtual void _step();

    // find and connect to downstreams
    virtual void _connect_to_downstreams();

    // ping model downstream to check if it is alive
    virtual bool _ping_model(std::shared_ptr<DownstreamModel> ds);

    // ping pipeline downstream to check if it is alive
    virtual bool _ping_pipeline(std::shared_ptr<DownstreamPipeline> ds);

    // send detections to all model downstreams
    virtual void _send_detections_to_downstreams();

    // send document to all pipeline downstreams
    virtual void _send_document_to_downstreams();

    virtual void _declare_all_parameters();

  protected:
    // member of pipeline downstreams
    std::map<std::string, std::shared_ptr<DownstreamPipeline>> m_pipeline_downstreams;

    // member of model downstreams
    std::map<std::string, std::shared_ptr<DownstreamModel>> m_model_downstreams;

    // action to be called by upstreams
    rclcpp_action::Server<ACT_AcceptDocument>::SharedPtr m_act_process_document;

    // configuration
    std::shared_ptr<InitConfig> m_init_config;
    std::shared_ptr<RuntimeConfig> m_runtime_config;

    // impl data
    std::shared_ptr<PoseDetectorInImpl> m_impl;


    // // on-going tasks of psg document processing
    // // indexed by (downstream, frame_number)
    Map_Document_Waiting m_psgdoc_task_waiting;
    Map_Document_Doing m_psgdoc_task_doing;
    Vec_Document_Done m_psgdoc_task_done;

    // // on-going tasks of frame processing
    // // indexed by (downstream, frame_number)
    Map_Detections_Waiting m_detections_task_waiting;
    Map_Detections_Doing m_detections_task_doing;
    Vec_Detections_Done m_detections_task_done;


    // status code
    int m_status_code = NodeStatusCode::BEFORE_INIT;
};
} // namespace FlowRos2Pipeline