#pragma once

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_nodes/base_nodes/OpenCloseNode.hpp>
#include <psg_tracker/AsyncGetTrackTargetsInputPort.hpp>
#include <psg_tracker/GetTrackTargetsInputSpec.hpp>
#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_cpp/redoxi_json_struct_conversion.hpp>
#include <json_struct/json_struct.h>
#include <RedoxiTrack/RedoxiTrack.h>
namespace redoxi_works
{
class PSGTracker;
class ROSTracker;

class TrackEventHandler : public RedoxiTrack::TrackingEventHandler
{
  public:
    /**
     * @brief Handles the event after target association in the tracker.
     *
     * This function is called when the target association event occurs in the tracker.
     * It updates the mapping between detections and targets in the m_det2target_associate map.
     *
     * @param sender A pointer to the TrackerBase object that triggered the event.
     * @param evt_data The TargetAssociation event data containing the detection and target information.
     * @return An integer value indicating the success of the function.
     */
    int evt_target_association_after(RedoxiTrack::TrackerBase *sender,
                                     const RedoxiTrack::TrackingEvent::TargetAssociation &evt_data) override
    {
        RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_association_after()");
        (void)sender;
        // auto &detection = evt_data.m_detection;
        auto &track_target = evt_data.m_target;
        track_target->set_path_state(RedoxiTrack::TrackPathStateBitmask::Open);

        // // test log
        // std::string str2 = "TrackTarget: {\n";
        // str2 += "track_id: " + std::to_string(track_target->get_path_id()) + "\n";
        // str2 += "track_status: " + std::to_string(track_target->get_path_state()) + "\n";
        // str2 += "track_bbox: Rect2f: {x: " + std::to_string(track_target->get_bbox().x) + ", y: " + std::to_string(track_target->get_bbox().y) + ", w: " + std::to_string(track_target->get_bbox().width) + ", h: " + std::to_string(track_target->get_bbox().height) + "}\n";
        // str2 += "}";
        // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_association_after(): %s", str2.c_str());

        m_det2target_associate[evt_data.m_detection] = evt_data.m_target;
        return 0;
    }


    /**
     * @brief Handles the event when a target is created after track.
     *
     * This function is called when a target is created after a detection event occurs.
     * It updates the mapping between the detection and the target in the m_det2target_create map.
     *
     * @param sender A pointer to the TrackerBase object that triggered the event.
     * @param evt_data The TargetAssociation event data containing the detection and target information.
     * @return An integer value indicating the success of the function.
     */
    int evt_target_created_after(RedoxiTrack::TrackerBase *sender,
                                 const RedoxiTrack::TrackingEvent::TargetAssociation &evt_data) override
    {
        RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_created_after()");
        (void)sender;
        // auto &detection = evt_data.m_detection;
        auto &track_target = evt_data.m_target;
        track_target->set_path_state(RedoxiTrack::TrackPathStateBitmask::New);

        // // test log
        // std::string str2 = "TrackTarget: {\n";
        // str2 += "track_id: " + std::to_string(track_target->get_path_id()) + "\n";
        // str2 += "track_status: " + std::to_string(track_target->get_path_state()) + "\n";
        // str2 += "track_bbox: Rect2f: {x: " + std::to_string(track_target->get_bbox().x) + ", y: " + std::to_string(track_target->get_bbox().y) + ", w: " + std::to_string(track_target->get_bbox().width) + ", h: " + std::to_string(track_target->get_bbox().height) + "}\n";
        // str2 += "}";
        // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_created_after(): %s", str2.c_str());

        m_det2target_create[evt_data.m_detection] = evt_data.m_target;
        // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_created_after(): m_det2target_create.size() = %ld", m_det2target_create.size());
        return 0;
    }

    /**
     * @brief Handles the event after a target is closed.
     *
     * This function is called when a target is closed in the tracker.
     * It updates the m_target_closed vector with the closed target.
     *
     * @param sender A pointer to the TrackerBase object that triggered the event.
     * @param evt_data The TargetClosed event data containing the closed target information.
     * @return An integer value indicating the success of the function.
     */
    int evt_target_closed_after(RedoxiTrack::TrackerBase *sender,
                                const RedoxiTrack::TrackingEvent::TargetClosed &evt_data) override
    {
        (void)sender;
        evt_data.m_target->set_path_state(RedoxiTrack::TrackPathStateBitmask::Close);
        m_target_closed.push_back(evt_data.m_target);
        return 0;
    }

    /**
     * @brief Handles the event after target motion prediction.
     *
     * This function is called when the target motion prediction event occurs in the tracker.
     * It updates the m_target_motion_predict vector with the predicted target.
     *
     * @param sender A pointer to the TrackerBase object that triggered the event.
     * @param evt_data The TargetMotionPredict event data containing the predicted target information.
     * @return An integer value indicating the success of the function.
     */
    int evt_target_motion_predict_after(RedoxiTrack::TrackerBase *sender,
                                        const RedoxiTrack::TrackingEvent::TargetMotionPredict &evt_data) override
    {
        (void)sender;
        m_target_motion_predict.push_back(evt_data.m_target);
        return 0;
    }

    void clear()
    {
        m_det2target_create.clear();
        m_det2target_associate.clear();
        m_target_closed.clear();
        m_target_motion_predict.clear();
    }

  public:
    /**
     * 检测对象与关联的跟踪对象
     */
    std::map<RedoxiTrack::DetectionPtr, RedoxiTrack::TrackTargetPtr> m_det2target_create;
    /**
     * 检测对象与关联的跟踪对象
     */
    std::map<RedoxiTrack::DetectionPtr, RedoxiTrack::TrackTargetPtr> m_det2target_associate;
    /**
     * 关闭的跟踪对象
     */
    std::vector<RedoxiTrack::TrackTargetPtr> m_target_closed;
    /**
     * 运动预测的跟踪对象
     */
    std::vector<RedoxiTrack::TrackTargetPtr> m_target_motion_predict;
};
using TrackEventHandlerPtr = std::shared_ptr<TrackEventHandler>;


namespace ROSTrackEvent
{
class ROSTrackEventData
{
  public:
    virtual ~ROSTrackEventData(){};
};
class ROSAssociation : public ROSTrackEventData
{
  public:
    int from_index;

    redoxi_public_msgs::msg::Detection from;
    redoxi_public_msgs::msg::TrackTarget to;
};

class ROSMotionPredict : public ROSTrackEventData
{
  public:
    /**
     * 运动预测的target
     */
    redoxi_public_msgs::msg::TrackTarget target;
};

class ROSClosed : public ROSTrackEventData
{
  public:
    /**
     * 关闭的target
     */
    redoxi_public_msgs::msg::TrackTarget target;
};

} // namespace ROSTrackEvent

class ROSTrackEventHandler
{
  public:
    virtual int evt_ROS_track_association_pre(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data)
    {
        (void)sender;
        (void)evt_data;
        return 0;
    };

    virtual int evt_ROS_track_association_after(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data)
    {
        (void)sender;
        (void)evt_data;
        return 0;
    };

    virtual int evt_ROS_track_create_pre(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data)
    {
        (void)sender;
        (void)evt_data;
        return 0;
    };

    virtual int evt_ROS_track_create_after(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data)
    {
        (void)sender;
        (void)evt_data;
        return 0;
    };

    virtual int evt_trajectory_closed_before(ROSTracker *sender, const ROSTrackEvent::ROSClosed &evt_data)
    {
        (void)sender;
        (void)evt_data;
        return 0;
    };

    virtual int evt_trajectory_closed_after(ROSTracker *sender, const ROSTrackEvent::ROSClosed &evt_data)
    {
        (void)sender;
        (void)evt_data;
        return 0;
    };

    virtual int evt_ROS_motion_predict_before(ROSTracker *sender, const ROSTrackEvent::ROSMotionPredict &evt_data)
    {
        (void)sender;
        (void)evt_data;
        return 0;
    };

    virtual int evt_ROS_motion_predict_after(ROSTracker *sender, const ROSTrackEvent::ROSMotionPredict &evt_data)
    {
        (void)sender;
        (void)evt_data;
        return 0;
    };
};
using ROSTrackEventHandlerPtr = std::shared_ptr<ROSTrackEventHandler>;


class MyROSTrackEventHandler : public ROSTrackEventHandler
{
  public:
    int evt_ROS_track_association_after(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data) override
    {
        (void)sender;
        m_target_associate.push_back(evt_data.to);
        return 0;
    };

    int evt_ROS_track_create_after(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data) override
    {
        (void)sender;
        m_target_create.push_back(evt_data.to);
        return 0;
    };

    int evt_trajectory_closed_after(ROSTracker *sender, const ROSTrackEvent::ROSClosed &evt_data) override
    {
        (void)sender;
        m_target_closed.push_back(evt_data.target);
        return 0;
    };

    int evt_ROS_motion_predict_after(ROSTracker *sender, const ROSTrackEvent::ROSMotionPredict &evt_data) override
    {
        (void)sender;
        m_target_motion_predict.push_back(evt_data.target);
        return 0;
    };

    void clear()
    {
        m_target_create.clear();
        m_target_associate.clear();
        m_target_closed.clear();
        m_target_motion_predict.clear();
    }

  public:
    std::vector<redoxi_public_msgs::msg::TrackTarget> m_target_create;

    std::vector<redoxi_public_msgs::msg::TrackTarget> m_target_associate;

    std::vector<redoxi_public_msgs::msg::TrackTarget> m_target_closed;

    std::vector<redoxi_public_msgs::msg::TrackTarget> m_target_motion_predict;
};
using MyROSTrackEventHandlerPtr = std::shared_ptr<MyROSTrackEventHandler>;


class ROSTracker
{
  public:
    virtual void init(const RedoxiTrack::TrackerBasePtr tracker_ptr, const TrackEventHandlerPtr track_event_handler);

    virtual void begin_track(const cv::Mat &img,
                             const std::vector<redoxi_public_msgs::msg::Detection> &detections,
                             int frame_number);

    virtual void track(const cv::Mat &img, const std::vector<redoxi_public_msgs::msg::Detection> &detections, int frame_number);

    virtual void track(const cv::Mat &img, int frame_number);

    virtual void finish_track();

    void add_event_handler(const ROSTrackEventHandlerPtr &person_event_handler);

    void remove_event_handler(const ROSTrackEventHandlerPtr &person_event_handler);

    const std::map<int, RedoxiTrack::TrackTargetPtr> get_all_open_targets() const
    {
        return m_tracked_targets;
    }

  private:
    void _msg_dets_to_body_dets(const std::vector<redoxi_public_msgs::msg::Detection> &detections,
                                std::vector<RedoxiTrack::DetectionPtr> &out_body_detections,
                                std::map<RedoxiTrack::DetectionPtr, std::pair<int, redoxi_public_msgs::msg::Detection>> &out_converted2before);
    redoxi_public_msgs::msg::TrackTarget _track_target_with_msg_det_to_msg(const redoxi_public_msgs::msg::Detection &detection, RedoxiTrack::TrackTargetPtr track_target);
    redoxi_public_msgs::msg::TrackTarget _track_target_to_msg(RedoxiTrack::TrackTargetPtr track_target);

    std::set<ROSTrackEventHandlerPtr> m_ros_track_event_handler_set;
    RedoxiTrack::TrackerBasePtr m_tracker;
    TrackEventHandlerPtr m_track_event_handler;

    std::map<int, RedoxiTrack::TrackTargetPtr> m_tracked_targets;
};
using ROSTrackerPtr = std::shared_ptr<ROSTracker>;


namespace psg_tracker
{
using InputPortType = AsyncGetTrackTargetsInputPort;

namespace TrackerTypes // FIXME: remove this namespace
{
const int DEEPSORT = 0;
const int BOTSORT = 1;
}; // namespace TrackerTypes

//! The init config for PSGTracker
struct InitConfig : public common_nodes::OpenCloseNode::InitConfig_t {
    virtual ~InitConfig() = default;
    InitConfig()
    {
        // default configuration for input port
        input_port_config->set_action_name("in/action");
        input_port_config->set_buffer_capacity(1);
    }

    std::shared_ptr<InputPortType::InitConfig_t>
        input_port_config = std::make_shared<InputPortType::InitConfig_t>();


    //! parse from node, the node must be exactly PSGPoseDetectorNode, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, PSGTracker>
    void from_node(const Node_t *node)
    {
        InitConfig::parse_from_node_parameters(this, node);
    }

    //! debug topics
    bool create_debug_pub = true;
    int debug_pub_queue_size = 10;
    std::string debug_topic_person_accepted = "debug_port/person_accepted";
    std::string debug_topic_person_rejected = "debug_port/person_rejected";

    //! tracker type
    int tracker_type = TrackerTypes::BOTSORT;

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::OpenCloseNode::InitConfig_t),
                         JS_MEMBER(input_port_config),
                         JS_MEMBER(create_debug_pub),
                         JS_MEMBER(debug_pub_queue_size),
                         JS_MEMBER(debug_topic_person_accepted),
                         JS_MEMBER(debug_topic_person_rejected),
                         JS_MEMBER(tracker_type));
};

//! The runtime config for PSGTracker
struct RuntimeConfig : public common_nodes::OpenCloseNode::RuntimeConfig_t {
  public:
    virtual ~RuntimeConfig() = default;
    RuntimeConfig()
    {
    }

    //! parse from node, the node must be exactly PSGTracker, not its subclass
    template <typename Node_t>
    requires std::is_same_v<Node_t, PSGTracker>
    void from_node(const Node_t *node)
    {
        RuntimeConfig::parse_from_node_parameters(this, node);
    }

    bool enable_blocking_mode = false;
    bool enable_debug_topics = true;

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::OpenCloseNode::RuntimeConfig_t),
                         JS_MEMBER(enable_blocking_mode),
                         JS_MEMBER(enable_debug_topics));
};

} // namespace psg_tracker

} // namespace redoxi_works
