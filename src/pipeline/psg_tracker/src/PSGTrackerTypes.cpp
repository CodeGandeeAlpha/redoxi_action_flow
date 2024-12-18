#include <psg_tracker/PSGTrackerTypes.hpp>
#include <psg_common/msg_converter.hpp>

namespace redoxi_works
{
void ROSTracker::init(const RedoxiTrack::TrackerBasePtr tracker_ptr, const TrackEventHandlerPtr track_event_handler)
{
    m_tracker = tracker_ptr;
    m_track_event_handler = track_event_handler;
}

void ROSTracker::begin_track(const cv::Mat &img,
                             const std::vector<redoxi_public_msgs::msg::Detection> &detections,
                             int frame_number)
{
    // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track()");
    std::vector<RedoxiTrack::DetectionPtr> det_bodies;
    std::map<RedoxiTrack::DetectionPtr, std::pair<int, redoxi_public_msgs::msg::Detection>> det_bodies2msg_detections;
    _msg_dets_to_body_dets(detections, det_bodies, det_bodies2msg_detections);
    // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track() after _msg_dets_to_body_dets()");

    // // test log
    // for (auto &det_body : det_bodies) {
    //     RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track() det_body %s",
    //                 psg_detection_to_string(det_body).c_str());
    // }

    m_track_event_handler->clear();
    // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track() before m_tracker->begin_track()");
    m_tracker->begin_track(img, det_bodies, frame_number);
    // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track() after m_tracker->begin_track()");
    for (auto &iter : m_track_event_handler->m_det2target_create) {
        auto track_det = iter.first;
        auto track_target = iter.second;
        auto detection_msg = det_bodies2msg_detections[track_det].second;

        // // test log
        // RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track(): create track target %s",
        //             psg_track_target_to_string(track_target).c_str());

        // create track target msg
        redoxi_public_msgs::msg::TrackTarget track_target_msg = _track_target_with_msg_det_to_msg(detection_msg, track_target);

        // create ROSAssociation data
        ROSTrackEvent::ROSAssociation evt_data;
        evt_data.from_index = det_bodies2msg_detections[track_det].first;
        evt_data.from = detection_msg;
        evt_data.to = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_ROS_track_create_pre(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_ROS_track_create_after(this, evt_data);
        }
    }

    m_tracked_targets = m_tracker->get_all_open_targets();
}


void ROSTracker::track(const cv::Mat &img, const std::vector<redoxi_public_msgs::msg::Detection> &detections, int frame_number)
{
    std::vector<RedoxiTrack::DetectionPtr> det_bodies;
    std::map<RedoxiTrack::DetectionPtr, std::pair<int, redoxi_public_msgs::msg::Detection>> det_bodies2msg_detections;
    RDX_INFO_DEV(nullptr, __func__, false, "{}", "正在将检测结果转换为内部格式");
    _msg_dets_to_body_dets(detections, det_bodies, det_bodies2msg_detections);

    // // test log
    // for (auto &det_body : det_bodies) {
    //     RCLCPP_DEBUG(rclcpp::get_logger("tracker_node"), "ROSTracker::track() det_body %s",
    //                 psg_detection_to_string(det_body).c_str());
    // }

    RDX_INFO_DEV(nullptr, __func__, false, "{}", "清除上一帧的事件处理器");
    m_track_event_handler->clear();

    RDX_INFO_DEV(nullptr, __func__, false, "{}", "开始跟踪处理");
    RDX_INFO_DEV(nullptr, __func__, false, "det_bodies.size() = {}", det_bodies.size());
    RDX_INFO_DEV(nullptr, __func__, false, "frame_number = {}", frame_number);
    m_tracker->track(img, det_bodies, frame_number);

    RDX_INFO_DEV(nullptr, __func__, false, "{}", "处理新创建的跟踪目标");
    for (auto &iter : m_track_event_handler->m_det2target_create) {
        auto track_det = iter.first;
        auto track_target = iter.second;
        auto detection_msg = det_bodies2msg_detections[track_det].second;

        // create track target msg
        redoxi_public_msgs::msg::TrackTarget track_target_msg = _track_target_with_msg_det_to_msg(detection_msg, track_target);

        // create ROSAssociation data
        ROSTrackEvent::ROSAssociation evt_data;
        evt_data.from_index = det_bodies2msg_detections[track_det].first;
        evt_data.from = detection_msg;
        evt_data.to = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_ROS_track_create_pre(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_ROS_track_create_after(this, evt_data);
        }
    }

    RDX_INFO_DEV(nullptr, __func__, false, "{}", "处理关联的跟踪目标");
    for (auto &iter : m_track_event_handler->m_det2target_associate) {
        auto track_det = iter.first;
        auto track_target = iter.second;
        auto detection_msg = det_bodies2msg_detections[track_det].second;

        // create track target msg
        redoxi_public_msgs::msg::TrackTarget track_target_msg = _track_target_with_msg_det_to_msg(detection_msg, track_target);

        // create ROSAssociation data
        ROSTrackEvent::ROSAssociation evt_data;
        evt_data.from_index = det_bodies2msg_detections[track_det].first;
        evt_data.from = detection_msg;
        evt_data.to = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_ROS_track_association_pre(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_ROS_track_association_after(this, evt_data);
        }
    }

    RDX_INFO_DEV(nullptr, __func__, false, "{}", "处理已关闭的跟踪目标");
    for (auto &target : m_track_event_handler->m_target_closed) {
        // create track target msg
        redoxi_public_msgs::msg::TrackTarget track_target_msg = _track_target_to_msg(target);

        // create ROSClosed data
        ROSTrackEvent::ROSClosed evt_data;
        evt_data.target = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_trajectory_closed_before(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_trajectory_closed_after(this, evt_data);
        }
    }

    RDX_INFO_DEV(nullptr, __func__, false, "{}", "更新当前活跃的跟踪目标");
    m_tracked_targets = m_tracker->get_all_open_targets();
}

void ROSTracker::track(const cv::Mat &img, int frame_number)
{
    m_track_event_handler->clear();
    m_tracker->track(img, frame_number);
    for (auto &target : m_track_event_handler->m_target_motion_predict) {
        // create track target msg
        redoxi_public_msgs::msg::TrackTarget track_target_msg = _track_target_to_msg(target);

        // create ROSMotionPredict data
        ROSTrackEvent::ROSMotionPredict evt_data;
        evt_data.target = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_ROS_motion_predict_before(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_ROS_motion_predict_after(this, evt_data);
        }
    }

    for (auto &target : m_track_event_handler->m_target_closed) {
        // create track target msg
        redoxi_public_msgs::msg::TrackTarget track_target_msg = _track_target_to_msg(target);

        // create ROSClosed data
        ROSTrackEvent::ROSClosed evt_data;
        evt_data.target = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_trajectory_closed_before(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_trajectory_closed_after(this, evt_data);
        }
    }

    m_tracked_targets = m_tracker->get_all_open_targets();
}

void ROSTracker::finish_track()
{
    m_track_event_handler->clear();
    m_tracker->finish_track();
    for (auto &target : m_track_event_handler->m_target_closed) {
        // create track target msg
        redoxi_public_msgs::msg::TrackTarget track_target_msg = _track_target_to_msg(target);

        // create ROSClosed data
        ROSTrackEvent::ROSClosed evt_data;
        evt_data.target = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_trajectory_closed_before(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            event_handler->evt_trajectory_closed_after(this, evt_data);
        }
    }

    m_tracked_targets = m_tracker->get_all_open_targets();
}

void ROSTracker::add_event_handler(const ROSTrackEventHandlerPtr &ros_event_handler)
{
    m_ros_track_event_handler_set.insert(ros_event_handler);
}

void ROSTracker::remove_event_handler(const ROSTrackEventHandlerPtr &ros_event_handler)
{
    m_ros_track_event_handler_set.erase(ros_event_handler);
}

void ROSTracker::_msg_dets_to_body_dets(const std::vector<redoxi_public_msgs::msg::Detection> &detections,
                                        std::vector<RedoxiTrack::DetectionPtr> &out_body_detections,
                                        std::map<RedoxiTrack::DetectionPtr, std::pair<int, redoxi_public_msgs::msg::Detection>> &out_converted2before)
{
    for (std::size_t i = 0; i < detections.size(); i++) {
        auto &det = detections[i];
        if (det.category != 0)
            continue;
        // RedoxiTrack::DetectionPtr body_det = std::make_shared<RedoxiTrack::Detection>();
        PassengerFlow::DetectionPtr body_det = std::make_shared<PassengerFlow::Detection>();
        FlowRos2Pipeline::convert_msg_to_detection(det, body_det);
        out_body_detections.push_back(body_det);
        out_converted2before[body_det] = std::make_pair(i, det);
    }
}

redoxi_public_msgs::msg::TrackTarget ROSTracker::_track_target_with_msg_det_to_msg(const redoxi_public_msgs::msg::Detection &detection, RedoxiTrack::TrackTargetPtr track_target)
{
    redoxi_public_msgs::msg::TrackTarget track_target_msg;
    track_target_msg.x_task_metadata = detection.x_task_metadata;
    track_target_msg.frame_metadata = detection.frame_metadata;
    track_target_msg.true_detection = detection;
    track_target_msg.track_id = track_target->get_path_id();
    track_target_msg.confidence = track_target->get_confidence();
    auto track_bbox = track_target->get_bbox();
    track_target_msg.track_bbox.x = track_bbox.x;
    track_target_msg.track_bbox.y = track_bbox.y;
    track_target_msg.track_bbox.width = track_bbox.width;
    track_target_msg.track_bbox.height = track_bbox.height;
    track_target_msg.track_status.bitmask = track_target->get_path_state();

    track_target_msg.predicted_detection.bbox = track_target_msg.track_bbox;
    track_target_msg.predicted_detection.category = 0;
    track_target_msg.predicted_detection.x_task_metadata = track_target_msg.x_task_metadata;
    track_target_msg.predicted_detection.frame_metadata = track_target_msg.frame_metadata;
    track_target_msg.predicted_detection.confidence = track_target_msg.confidence;
    return track_target_msg;
}

redoxi_public_msgs::msg::TrackTarget ROSTracker::_track_target_to_msg(RedoxiTrack::TrackTargetPtr track_target)
{
    redoxi_public_msgs::msg::TrackTarget track_target_msg;
    track_target_msg.track_id = track_target->get_path_id();
    track_target_msg.confidence = track_target->get_confidence();
    auto track_bbox = track_target->get_bbox();
    track_target_msg.track_bbox.x = track_bbox.x;
    track_target_msg.track_bbox.y = track_bbox.y;
    track_target_msg.track_bbox.width = track_bbox.width;
    track_target_msg.track_bbox.height = track_bbox.height;
    track_target_msg.track_status.bitmask = track_target->get_path_state();
    track_target_msg.frame_metadata.frame_num = track_target->get_end_frame_number();

    track_target_msg.predicted_detection.bbox = track_target_msg.track_bbox;
    track_target_msg.predicted_detection.category = 0;
    track_target_msg.predicted_detection.x_task_metadata = track_target_msg.x_task_metadata;
    track_target_msg.predicted_detection.frame_metadata = track_target_msg.frame_metadata;
    track_target_msg.predicted_detection.confidence = track_target_msg.confidence;

    return track_target_msg;
}
} // namespace redoxi_works