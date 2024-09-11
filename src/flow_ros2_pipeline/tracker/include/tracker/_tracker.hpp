#pragma once

#include <RedoxiTrack/RedoxiTrack.h>
#include <boost/thread/synchronized_value.hpp>
#include <memory>
#include <thread>
#include <tracker/tracker.hpp>
#include <vineyard/client/client.h>
#include <vineyard/client/ds/object_meta.h>

namespace FlowRos2Pipeline
{
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

        auto &detection = evt_data.m_detection;
        auto &track_target = evt_data.m_target;
        track_target->set_path_state(RedoxiTrack::TrackPathStateBitmask::Open);

        // // test log
        // std::string str = "Detection: {\n";
        // str += "category: " + std::to_string(detection->get_type()) + "\n";
        // str += "bbox: Rect2f: {x: " + std::to_string(detection->get_bbox().x) + ", y: " + std::to_string(detection->get_bbox().y) + ", w: " + std::to_string(detection->get_bbox().width) + ", h: " + std::to_string(detection->get_bbox().height) + "}\n";
        // str += "confidence: " + std::to_string(detection->get_confidence()) + "\n";
        // str += "}";
        // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_association_after(): %s", str.c_str());

        std::string str2 = "TrackTarget: {\n";
        str2 += "track_id: " + std::to_string(track_target->get_path_id()) + "\n";
        str2 += "track_status: " + std::to_string(track_target->get_path_state()) + "\n";
        str2 += "track_bbox: Rect2f: {x: " + std::to_string(track_target->get_bbox().x) + ", y: " + std::to_string(track_target->get_bbox().y) + ", w: " + std::to_string(track_target->get_bbox().width) + ", h: " + std::to_string(track_target->get_bbox().height) + "}\n";
        str2 += "}";
        RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_association_after(): %s", str2.c_str());

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

        auto &detection = evt_data.m_detection;
        auto &track_target = evt_data.m_target;
        track_target->set_path_state(RedoxiTrack::TrackPathStateBitmask::New);

        // // test log
        // std::string str = "Detection: {\n";
        // str += "category: " + std::to_string(detection->get_type()) + "\n";
        // str += "bbox: Rect2f: {x: " + std::to_string(detection->get_bbox().x) + ", y: " + std::to_string(detection->get_bbox().y) + ", w: " + std::to_string(detection->get_bbox().width) + ", h: " + std::to_string(detection->get_bbox().height) + "}\n";
        // str += "confidence: " + std::to_string(detection->get_confidence()) + "\n";
        // str += "}";
        // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_created_after(): %s", str.c_str());

        std::string str2 = "TrackTarget: {\n";
        str2 += "track_id: " + std::to_string(track_target->get_path_id()) + "\n";
        str2 += "track_status: " + std::to_string(track_target->get_path_state()) + "\n";
        str2 += "track_bbox: Rect2f: {x: " + std::to_string(track_target->get_bbox().x) + ", y: " + std::to_string(track_target->get_bbox().y) + ", w: " + std::to_string(track_target->get_bbox().width) + ", h: " + std::to_string(track_target->get_bbox().height) + "}\n";
        str2 += "}";
        RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_created_after(): %s", str2.c_str());

        m_det2target_create[evt_data.m_detection] = evt_data.m_target;
        RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "evt_target_created_after(): m_det2target_create.size() = %ld", m_det2target_create.size());
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

    Tracker::MSG_Detection from;
    Tracker::MSG_TrackTarget to;
};

class ROSMotionPredict : public ROSTrackEventData
{
  public:
    /**
     * 运动预测的target
     */
    Tracker::MSG_TrackTarget target;
};

class ROSClosed : public ROSTrackEventData
{
  public:
    /**
     * 关闭的target
     */
    Tracker::MSG_TrackTarget target;
};

} // namespace ROSTrackEvent

class ROSTrackEventHandler
{
  public:
    virtual int evt_ROS_track_association_pre(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data)
    {
        return 0;
    };

    virtual int evt_ROS_track_association_after(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data)
    {
        return 0;
    };

    virtual int evt_ROS_track_create_pre(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data)
    {
        return 0;
    };

    virtual int evt_ROS_track_create_after(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data)
    {
        return 0;
    };

    virtual int evt_trajectory_closed_before(ROSTracker *sender, const ROSTrackEvent::ROSClosed &evt_data)
    {
        return 0;
    };

    virtual int evt_trajectory_closed_after(ROSTracker *sender, const ROSTrackEvent::ROSClosed &evt_data)
    {
        return 0;
    };

    virtual int evt_ROS_motion_predict_before(ROSTracker *sender, const ROSTrackEvent::ROSMotionPredict &evt_data)
    {
        return 0;
    };

    virtual int evt_ROS_motion_predict_after(ROSTracker *sender, const ROSTrackEvent::ROSMotionPredict &evt_data)
    {
        return 0;
    };
};
using ROSTrackEventHandlerPtr = std::shared_ptr<ROSTrackEventHandler>;


class MyROSTrackEventHandler : public ROSTrackEventHandler
{
  public:
    int evt_ROS_track_association_after(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data) override
    {
        m_target_associate.push_back(evt_data.to);
        return 0;
    };

    int evt_ROS_track_create_after(ROSTracker *sender, const ROSTrackEvent::ROSAssociation &evt_data) override
    {
        m_target_create.push_back(evt_data.to);
        return 0;
    };

    int evt_trajectory_closed_after(ROSTracker *sender, const ROSTrackEvent::ROSClosed &evt_data) override
    {
        m_target_closed.push_back(evt_data.target);
        return 0;
    };

    int evt_ROS_motion_predict_after(ROSTracker *sender, const ROSTrackEvent::ROSMotionPredict &evt_data) override
    {
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
    std::vector<Tracker::MSG_TrackTarget> m_target_create;

    std::vector<Tracker::MSG_TrackTarget> m_target_associate;

    std::vector<Tracker::MSG_TrackTarget> m_target_closed;

    std::vector<Tracker::MSG_TrackTarget> m_target_motion_predict;
};
using MyROSTrackEventHandlerPtr = std::shared_ptr<MyROSTrackEventHandler>;


class ROSTracker
{
  public:
    virtual void init(const RedoxiTrack::TrackerBasePtr tracker_ptr, const TrackEventHandlerPtr track_event_handler);

    virtual void begin_track(const cv::Mat &img,
                             const Tracker::MSG_Detections &detections,
                             int frame_number);

    virtual void track(const cv::Mat &img, const Tracker::MSG_Detections &detections, int frame_number);

    virtual void track(const cv::Mat &img, int frame_number);

    virtual void finish_track();

    void add_event_handler(const ROSTrackEventHandlerPtr &person_event_handler);

    void remove_event_handler(const ROSTrackEventHandlerPtr &person_event_handler);

    const std::map<int, RedoxiTrack::TrackTargetPtr> get_all_open_targets() const
    {
        return m_tracked_targets;
    }

  private:
    void _msg_dets_to_body_dets(const Tracker::MSG_Detections &detections,
                                std::vector<RedoxiTrack::DetectionPtr> &out_body_detections,
                                std::map<RedoxiTrack::DetectionPtr, std::pair<int, Tracker::MSG_Detection>> &out_converted2before);
    Tracker::MSG_TrackTarget _track_target_with_msg_det_to_msg(const Tracker::MSG_Detection &detection, RedoxiTrack::TrackTargetPtr track_target);
    Tracker::MSG_TrackTarget _track_target_to_msg(RedoxiTrack::TrackTargetPtr track_target);

    std::set<ROSTrackEventHandlerPtr> m_ros_track_event_handler_set;
    RedoxiTrack::TrackerBasePtr m_tracker;
    TrackEventHandlerPtr m_track_event_handler;

    std::map<int, RedoxiTrack::TrackTargetPtr> m_tracked_targets;
};
using ROSTrackerPtr = std::shared_ptr<ROSTracker>;


class TrackerImpl
{
  public:
    virtual ~TrackerImpl()
    {
    }
    TrackerImpl(Tracker *node)
        : logger(node->get_logger())
    {
    }
    rclcpp::Logger logger;
    std::shared_ptr<vineyard::Client> v6d_client;

    boost::synchronized_value<Tracker::Map_TrackTargets_Waiting *> sync_track_targets_waiting_map;
    boost::synchronized_value<Tracker::Map_TrackTargets_Doing *> sync_track_targets_doing_map;

    boost::synchronized_value<Tracker::Map_Detections *> sync_detections_map;
    boost::synchronized_value<Tracker::Map_TrackTargets *> sync_track_targets_map;

    ROSTrackerPtr tracker;
    MyROSTrackEventHandlerPtr ros_track_event_handler;

    cv::VideoWriter out_video_writer;

    std::shared_ptr<std::thread> step_thread;
    std::shared_ptr<std::thread> process_thread;
    bool step_running = false; // for stopping the step thread

    bool visualize_flag = false;
};
} // namespace FlowRos2Pipeline