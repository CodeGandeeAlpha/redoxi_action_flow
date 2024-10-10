#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <psg_actions/action/process_detections.hpp>
#include <psg_actions/action/process_frame.hpp>
#include <psg_actions/action/process_psg_document.hpp>
#include <psg_common/psg_common.hpp>
#include <psg_services/srv/status_query.hpp>

#include <detector/pipeline_types.hpp>


namespace FlowRos2Pipeline
{
class DetectorPipelineImpl;

class DetectorPipeline : public rclcpp::Node, public IStartStopProtocol
{
  public:
    class DownstreamPipeline;
    class DownstreamModel;
    class DSTask_PsgDocument;
    class DSTask_Frame;

  public:
    using ACT_AcceptFrame = psg_actions::action::ProcessFrame;
    using ACT_AcceptDocument = psg_actions::action::ProcessPsgDocument;
    using ACT_AcceptDetections = psg_actions::action::ProcessDetections;

    using InitConfig = DetectorPipelineInitConfig;
    using RuntimeConfig = DetectorPipelineRuntimeConfig;
    using MSG_Frame = psg_public_msgs::msg::Frame;
    using MSG_Detection = psg_public_msgs::msg::Detection;
    using MSG_Detections = psg_public_msgs::msg::Detections;
    using MSG_PsgDocument = psg_private_msgs::msg::PsgDocument;
    using MSG_UUID = unique_identifier_msgs::msg::UUID;

    using GoalHandle_PsgDocument = rclcpp_action::ClientGoalHandle<ACT_AcceptDocument>::SharedPtr;
    using GoalHandle_Frame = rclcpp_action::ClientGoalHandle<ACT_AcceptFrame>::SharedPtr;

    // 针对每个下游的任务，key是downstream和frame_number，value是任务
    // 每个消息会变成多个任务，他们的消息内容是一样的，但是下游不一样
    using Map_Document_Waiting = std::map<std::tuple<DownstreamPipeline *, int>, std::shared_ptr<DSTask_PsgDocument>>;
    using Map_Document_Doing = std::map<GoalHandle_PsgDocument, std::shared_ptr<DSTask_PsgDocument>>;
    using Vec_Document_Done = std::vector<std::shared_ptr<DSTask_PsgDocument>>;
    using Map_Frame_Waiting = std::map<std::tuple<DownstreamModel *, int>, std::shared_ptr<DSTask_Frame>>;
    using Map_Frame_Doing = std::map<GoalHandle_Frame, std::shared_ptr<DSTask_Frame>>;
    using Vec_Frame_Done = std::vector<std::shared_ptr<DSTask_Frame>>;

    using Map_Task_Waiting = std::map<int, std::pair<std::shared_ptr<DSTask_PsgDocument>, std::shared_ptr<DSTask_Frame>>>;

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
        rclcpp_action::Client<ACT_AcceptFrame>::SharedPtr accept_frame;
        rclcpp_action::Client<ACT_AcceptFrame>::SendGoalOptions accept_frame_options;
    };


    class DSTask_Frame
    {
      public:
        MSG_Frame frame; // frame associated with this task
        std::shared_ptr<DownstreamModel> downstream;
        MSG_UUID detections_uuid;
        GoalHandle_Frame goal_handle; // downstream goal handle
        int retry_times = 0;          // retry times already
    };

    class DSTask_PsgDocument
    {
      public:
        MSG_PsgDocument document; // frame associated with this task
        std::shared_ptr<DownstreamPipeline> downstream;
        GoalHandle_PsgDocument goal_handle; // downstream goal handle
        int retry_times = 0;                // retry times already
    };

  public:
    explicit DetectorPipeline();

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

    // accept model results from model downstreams
    rclcpp_action::Server<ACT_AcceptDetections>::SharedPtr m_act_accept_model_results;
    virtual rclcpp_action::GoalResponse _accept_model_results_goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ACT_AcceptDetections::Goal> goal);
    virtual rclcpp_action::CancelResponse _accept_model_results_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle);
    virtual void _accept_model_results_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle);

  protected:
    // create tasks
    virtual void _process_document_create_tasks(const MSG_PsgDocument &document, Map_Document_Waiting *document_waiting_map_ptr);
    virtual void _process_frame_create_tasks(const MSG_Frame &frame, const MSG_UUID &detections_uuid, Map_Frame_Waiting *frame_waiting_map_ptr);

    // add document to buffer
    virtual void _add_document_to_buffer(const MSG_PsgDocument &document, std::map<int, MSG_PsgDocument> *document_buffer_ptr);
    // remove document from buffer
    virtual void _remove_document_from_buffer(int frame_number, std::map<int, MSG_PsgDocument> *document_buffer_ptr);

    // add detections to buffer
    virtual void _add_detections_to_buffer(const MSG_Detections &detections, std::map<int, std::vector<MSG_Detections>> *detections_buffer_ptr);
    // remove detections from buffer
    virtual void _remove_detections_from_buffer(int frame_number, std::map<int, std::vector<MSG_Detections>> *detections_buffer_ptr);

  protected:
    virtual void _step();
    virtual void _process_step();

    // find and connect to downstreams
    virtual void _connect_to_downstreams();

    // ping model downstream to check if it is alive
    virtual bool _ping_model(std::shared_ptr<DownstreamModel> ds);

    // ping pipeline downstream to check if it is alive
    virtual bool _ping_pipeline(std::shared_ptr<DownstreamPipeline> ds);

    // send frame to all model downstreams
    virtual void _send_frame_to_downstreams();

    // send document to all pipeline downstreams
    virtual void _send_document_to_downstreams();

    // merge detections and documents
    virtual void _merge_detections_and_documents();

    // declare all parameters
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
    std::shared_ptr<DetectorPipelineImpl> m_impl;


    // // on-going tasks of psg document processing
    // // indexed by (downstream, frame_number)
    Map_Document_Waiting m_psgdoc_task_waiting;
    Map_Document_Doing m_psgdoc_task_doing;
    Vec_Document_Done m_psgdoc_task_done;

    // // on-going tasks of frame processing
    // // indexed by (downstream, frame_number)
    Map_Frame_Waiting m_frame_task_waiting;
    // Map_Frame_Doing m_frame_task_doing;
    // Vec_Frame_Done m_frame_task_done;

    // buffer
    std::map<int, MSG_PsgDocument> m_document_buffer;               // indexed by frame number
    std::map<int, std::vector<MSG_Detections>> m_detections_buffer; // indexed by frame number

    // status code
    int m_status_code = NodeStatusCode::BEFORE_INIT;
};
} // namespace FlowRos2Pipeline