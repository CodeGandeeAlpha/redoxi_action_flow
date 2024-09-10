#pragma once

#include <map>
#include <memory>
#include <psg_actions/action/process_detections.hpp>
#include <psg_actions/action/process_track_targets.hpp>
#include <psg_common/psg_common.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tracker/tracker_types.hpp>


namespace FlowRos2Pipeline
{
class TrackerImpl;
class TrackingEventHandler;
class ROSTracker;

class Tracker : public rclcpp::Node, public IOpenCloseProtocol
{
  public:
    class Downstream;
    class DSTask_TrackTargets;

  public:
    using ACT_AcceptTrackTargets = psg_actions::action::ProcessTrackTargets;
    using ACT_AcceptDetections = psg_actions::action::ProcessDetections;

    using InitConfig = TrackerInitConfig;
    using RuntimeConfig = TrackerRuntimeConfig;
    using MSG_Frame = psg_public_msgs::msg::Frame;
    using MSG_Detection = psg_public_msgs::msg::Detection;
    using MSG_Detections = psg_public_msgs::msg::Detections;
    using MSG_TrackTarget = psg_public_msgs::msg::TrackTarget;
    using MSG_TrackTargets = ACT_AcceptTrackTargets::Goal::_track_targets_type;

    using GoalHandle_TrackTargets = rclcpp_action::ClientGoalHandle<ACT_AcceptTrackTargets>::SharedPtr;

    using Map_TrackTargets_Waiting = std::map<std::tuple<Downstream *, int>, std::shared_ptr<DSTask_TrackTargets>>;
    using Map_TrackTargets_Doing = std::map<GoalHandle_TrackTargets, std::shared_ptr<DSTask_TrackTargets>>;
    using Vec_TrackTargets_Done = std::vector<std::shared_ptr<DSTask_TrackTargets>>;

    using Map_Detections = std::map<int, MSG_Detections>;
    using Map_TrackTargets = std::map<int, std::tuple<MSG_TrackTargets, MSG_Frame>>;

    class Downstream
    {
      public:
        virtual ~Downstream()
        {
        }
        // client to call action
        rclcpp_action::Client<ACT_AcceptTrackTargets>::SharedPtr accept_track_targets;
        rclcpp_action::Client<ACT_AcceptTrackTargets>::SendGoalOptions accept_track_targets_options;
    };

    class DSTask_TrackTargets
    {
      public:
        MSG_TrackTargets track_targets; // track targets associated with this task
        MSG_Frame frame;                // frame associated with this task
        std::shared_ptr<Downstream> downstream;
        GoalHandle_TrackTargets goal_handle; // downstream goal handle
    };

  public:
    explicit Tracker();

    // initialize with configurations, must be called once before open()
    virtual int init(const std::shared_ptr<InitConfig> &config, const std::shared_ptr<RuntimeConfig> &runtime_config);

    // you can set configuration before open() or after close()
    virtual const std::shared_ptr<InitConfig> &get_init_config() const;

    // modify runtime settings, must be called before start(), after stop() or close()
    virtual int update_runtime_config(const std::shared_ptr<RuntimeConfig> &config);
    virtual const std::shared_ptr<RuntimeConfig> &get_runtime_config() const;

    // can modify init config, runtime config

    // open video source, get ready to read
    virtual int open() override;

    // can modify runtime config

    // call this after ready() and before you spin this node
    // after calling this, you cannot modify runtime config
    virtual int start() override;

    // cannot modify any config, can call set_xxx() to modify relevant states

    // call this before you modify runtime config
    virtual int stop() override;

    // can modify runtime config

    // call this before you want to modify init config
    virtual int close() override;

    // can modify init config, runtime config

    // get the status code of this node
    virtual int get_status_code() const;

  protected:
    // accept detections from upstream
    virtual rclcpp_action::GoalResponse _accept_detections_goal_callback(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ACT_AcceptDetections::Goal> goal);
    virtual rclcpp_action::CancelResponse _accept_detections_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle);
    virtual void _accept_detections_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle);

    // create tasks
    virtual void _process_track_targets_create_tasks(const MSG_TrackTargets &track_targets,
                                                     const MSG_Frame &frame, Map_TrackTargets_Waiting *track_targets_waiting_map_ptr);

    // add buffer
    virtual void _add_detections_to_buffer(const MSG_Detections &detections, Map_Detections *detections_map_ptr);
    virtual void _add_track_targets_to_buffer(const MSG_TrackTargets &track_targets, MSG_Frame &frame, Map_TrackTargets *track_targets_map_ptr);

    // remove buffer
    virtual void _remove_detections_from_buffer(int frame_number, Map_Detections *detections_map_ptr);
    virtual void _remove_track_targets_from_buffer(int frame_number, Map_TrackTargets *track_targets_map_ptr);

  protected:
    virtual void _step();
    virtual void _process_step();

    // find and connect to downstreams
    virtual void _connect_to_downstreams();

    // send document to all pipeline downstreams
    virtual void _send_track_targets_to_downstreams();

    virtual void _declare_all_parameters();

  protected:
    // member of pipeline downstreams
    std::map<std::string, std::shared_ptr<Downstream>> m_downstreams;

    // action to be called by upstreams
    rclcpp_action::Server<ACT_AcceptDetections>::SharedPtr m_act_process_detections;

    // configuration
    std::shared_ptr<InitConfig> m_init_config;
    std::shared_ptr<RuntimeConfig> m_runtime_config;

    // impl data
    std::shared_ptr<TrackerImpl> m_impl;

    // on-going tasks of psg document processing
    // indexed by (downstream, frame_number)
    Map_TrackTargets_Waiting m_track_targets_task_waiting;
    Map_TrackTargets_Doing m_track_targets_task_doing;
    Vec_TrackTargets_Done m_track_targets_task_done;

    // buffer for detections
    Map_Detections m_detections_buffer;
    // buffer for track targets
    Map_TrackTargets m_track_targets_buffer;

    // waiting framenumer
    int m_waiting_frame_number = 0;


    // status code
    int m_status_code = NodeStatusCode::BEFORE_INIT;
};
} // namespace FlowRos2Pipeline