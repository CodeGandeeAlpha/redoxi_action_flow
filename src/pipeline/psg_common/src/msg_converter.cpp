#include <psg_common/msg_converter.hpp>

namespace FlowRos2Pipeline
{
void convert_detection_to_msg(PassengerFlow::Detection &det, const redoxi_public_msgs::msg::Frame &msg_frame, redoxi_public_msgs::msg::Detection &msg)
{
    // msg Detection {0: body, 1: head, 2: face} PassengerFlow::Detection {0: None, 1: head, 2: face, 3:body}
    std::vector<int> class_id_mapping = {-1, 1, 2, 0};
    std::vector<std::string> category_name_mapping = {"None", "head", "face", "body"};

    auto bbox = det.get_bbox();
    msg.bbox.x = bbox.x;
    msg.bbox.y = bbox.y;
    msg.bbox.width = bbox.width;
    msg.bbox.height = bbox.height;
    msg.category = class_id_mapping[det.get_type()];
    msg.category_name = category_name_mapping[det.get_type()];
    msg.confidence = det.get_confidence();
    msg.frame_metadata = msg_frame.metadata;
    // msg.is_detected_by_camera = det.detected_by_camera();

    if (det.get_type() == RedoxiTrack::DetectionTypes::PersonBody) {
        RedoxiTrack::fVECTOR feature;
        det.get_feature(feature);
        std::vector<double> vec(feature.data(), feature.data() + feature.rows() * feature.cols());
        msg.feature = vec;
    }
}


void convert_msg_to_detection(const redoxi_public_msgs::msg::Detection &msg, PassengerFlow::DetectionPtr &det)
{
    auto bbox = PassengerFlow::BBOX(msg.bbox.x, msg.bbox.y, msg.bbox.width, msg.bbox.height);
    det->set_bbox(bbox);
    det->set_confidence(msg.confidence);

    // msg.category {body, head, face}
    if (msg.category == 0) {
        RedoxiTrack::fVECTOR single_feature;
        single_feature.resize(128, 1);
        for (size_t j = 0; j < msg.feature.size(); j++) {
            single_feature(j, 0) = msg.feature[j];
        }
        det->set_type(RedoxiTrack::DetectionTypes::PersonBody);
        det->set_feature(single_feature);
    } else if (msg.category == 2) {
        det->set_type(RedoxiTrack::DetectionTypes::PersonFace);
    } else {
        det->set_type(RedoxiTrack::DetectionTypes::PersonHead);
    }

    det->set_frame_number(msg.frame_metadata.frame_num);
    // det->set_detected_by_camera(msg.is_detected_by_camera);
}


// void convert_detections_to_msg(const std::vector<PassengerFlow::DetectionPtr> &dets, const redoxi_public_msgs::msg::Frame &msg_frame, redoxi_public_msgs::msg::Detections &msg)
// {
//     for (auto &det : dets) {
//         redoxi_public_msgs::msg::Detection msg_det;
//         convert_detection_to_msg(*det, msg_frame, msg_det);
//         msg.detections.push_back(msg_det);
//         msg.frame = msg_frame;
//     }
// }


// void convert_msg_to_detections(const redoxi_public_msgs::msg::Detections &msg, std::vector<PassengerFlow::DetectionPtr> &dets)
// {
//     for (auto &msg_det : msg.detections) {
//         PassengerFlow::DetectionPtr det = std::make_shared<PassengerFlow::Detection>();
//         convert_msg_to_detection(msg_det, det);
//         dets.push_back(det);
//     }
// }


void convert_person_to_msg(const PassengerFlow::PersonPtr &person, const redoxi_public_msgs::msg::Frame &msg_frame, psg_private_msgs::msg::Person &msg)
{
    msg.true_body.category = -1;
    msg.true_face.category = -1;
    msg.true_head.category = -1;

    msg.body.category = -1;
    msg.face.category = -1;
    msg.head.category = -1;

    if (person->body()) {
        redoxi_public_msgs::msg::Detection msg_body;
        auto *body_ptr = RedoxiTrack::dyncast_with_check<PassengerFlow::Detection>(person->body().get());
        convert_detection_to_msg(*body_ptr, msg_frame, msg_body);
        msg.true_body = msg_body;
    }
    if (person->head()) {
        redoxi_public_msgs::msg::Detection msg_head;
        auto *head_ptr = RedoxiTrack::dyncast_with_check<PassengerFlow::Detection>(person->head().get());
        convert_detection_to_msg(*head_ptr, msg_frame, msg_head);
        msg.true_head = msg_head;
    }
    if (person->face()) {
        redoxi_public_msgs::msg::Detection msg_face;
        auto *face_ptr = RedoxiTrack::dyncast_with_check<PassengerFlow::Detection>(person->face().get());
        convert_detection_to_msg(*face_ptr, msg_frame, msg_face);
        msg.true_face = msg_face;
    }

    auto person_pose = person->get_keypoints();
    if (person_pose.size() > 0) {
        redoxi_public_msgs::msg::Keypoints msg_body_pose;
        for (auto &kp : person_pose) {
            auto kp_type = kp.first;
            msg_body_pose.semantic_type.push_back(kp_type);
            msg_body_pose.confidence.push_back(kp.second.m_confidence);
            // msg_pose.x.push_back(kp.second.m_point.x);
            // msg_pose.y.push_back(kp.second.m_point.y);
            geometry_msgs::msg::Point pt;
            pt.x = kp.second.m_point.x;
            pt.y = kp.second.m_point.y;
            pt.z = 0;
            msg_body_pose.keypoints_2.push_back(pt);
        }
        msg.true_body.keypoints = msg_body_pose;
    }

    msg.track_id = person->get_person_id();
    msg.frame_metadata = msg_frame.metadata;
    msg.body_height = person->get_body_height().m_body_height;
    msg.body_height_conf = person->get_body_height().m_body_height_conf;
    PassengerFlow::POINT3 position;
    bool position_valid;
    person->get_foot_position(&position, &position_valid);
    if (position_valid) {
        geometry_msgs::msg::Point pt;
        pt.x = position.x;
        pt.y = position.y;
        pt.z = position.z;
        msg.foot_position_3 = pt;
    }
    person->get_head_position(&position, &position_valid);
    if (position_valid) {
        geometry_msgs::msg::Point pt;
        pt.x = position.x;
        pt.y = position.y;
        pt.z = position.z;
        msg.head_position_3 = pt;
    }
}


void convert_msg_to_person(const psg_private_msgs::msg::Person &msg, PassengerFlow::PersonPtr &person)
{
    if (msg.true_body.category != -1) {
        PassengerFlow::DetectionPtr one_det = std::make_shared<PassengerFlow::Detection>();
        convert_msg_to_detection(msg.true_body, one_det);
        person->set_body(one_det);
    }

    if (msg.true_head.category != -1) {
        PassengerFlow::DetectionPtr one_det = std::make_shared<PassengerFlow::Detection>();
        convert_msg_to_detection(msg.true_head, one_det);
        person->set_head(one_det);
    }

    if (msg.true_face.category != -1) {
        PassengerFlow::DetectionPtr one_det = std::make_shared<PassengerFlow::Detection>();
        convert_msg_to_detection(msg.true_face, one_det);
        person->set_face(one_det);
    }

    if (msg.true_body.keypoints.keypoints_2.size() > 0) {
        std::map<PassengerFlow::KeyPointSemanticType, PassengerFlow::Keypoint> pose;
        for (size_t i = 0; i < msg.true_body.keypoints.confidence.size(); i++) {
            PassengerFlow::KeyPointSemanticType pt_type = msg.true_body.keypoints.semantic_type[i];
            PassengerFlow::Keypoint kpt;
            kpt.m_confidence = msg.true_body.keypoints.confidence[i];
            auto &pt = msg.true_body.keypoints.keypoints_2[i];
            kpt.m_point.x = pt.x;
            kpt.m_point.y = pt.y;
            kpt.m_semantic_id = pt_type;
            pose[pt_type] = kpt;
        }
        person->set_keypoints(pose);
    }

    PassengerFlow::BodyHeight height;
    height.m_body_height = msg.body_height;
    height.m_body_height_conf = msg.body_height_conf;
    person->set_body_height(height);

    if (msg.foot_position_3.x != 0 || msg.foot_position_3.y != 0 || msg.foot_position_3.z != 0) {
        person->set_foot_position({float(msg.foot_position_3.x), float(msg.foot_position_3.y), float(msg.foot_position_3.z)});
    }

    if (msg.head_position_3.x != 0 || msg.head_position_3.y != 0 || msg.head_position_3.z != 0) {
        person->set_head_position({float(msg.head_position_3.x), float(msg.head_position_3.y), float(msg.head_position_3.z)});
    }

    person->set_person_id(msg.track_id);
    person->set_frame_number(msg.frame_metadata.frame_num);
}


// void convert_persons_to_msg(const std::vector<PassengerFlow::PersonPtr> &persons, const redoxi_public_msgs::msg::Frame &msg_frame, psg_private_msgs::msg::Persons &msg)
// {
//     msg.frame = msg_frame;
//     for (auto &person : persons) {
//         psg_private_msgs::msg::Person msg_person;
//         msg_person.frame = msg_frame;
//         convert_person_to_msg(person, msg_frame, msg_person);
//         msg.persons.push_back(msg_person);
//     }
// }


// void convert_msg_to_persons(const psg_private_msgs::msg::Persons &msg, std::vector<PassengerFlow::PersonPtr> &persons)
// {
//     for (auto &msg_person : msg.persons) {
//         auto person_ptr = std::make_shared<PassengerFlow::Person>();
//         convert_msg_to_person(msg_person, person_ptr);
//         persons.push_back(person_ptr);
//     }
// }


void convert_traj_to_msg(const PassengerFlow::PersonTrajectory &traj, psg_private_msgs::msg::PersonTrajectory &msg)
{
    int person_id = traj.get_person_id();
    msg.track_id = person_id;

    for (auto &person : traj.m_person_list) {
        psg_private_msgs::msg::Person msg_person;
        redoxi_public_msgs::msg::Frame msg_frame;
        msg_frame.metadata.frame_num = person->get_frame_number();
        convert_person_to_msg(person, msg_frame, msg_person);
        msg.persons.push_back(msg_person);
    }
}


void convert_msg_to_traj(const psg_private_msgs::msg::PersonTrajectory &msg, PassengerFlow::PersonTrajectory &traj)
{
    for (auto &msg_person : msg.persons) {
        PassengerFlow::PersonPtr person = std::make_shared<PassengerFlow::Person>();
        convert_msg_to_person(msg_person, person);
        person->set_person_id(msg.track_id);
        traj.insert(person);
    }
}


// void convert_trajs_to_msg(const std::vector<PassengerFlow::PersonTrajectory> &trajs, psg_private_msgs::msg::PersonTrajectories &msg)
// {
//     for (auto &traj : trajs) {
//         psg_private_msgs::msg::PersonTrajectory msg_traj;
//         convert_traj_to_msg(traj, msg_traj);
//         msg.person_trajectories.push_back(msg_traj);
//     }
// }


// void convert_msg_to_trajs(const psg_private_msgs::msg::PersonTrajectories &msg, std::vector<PassengerFlow::PersonTrajectory> &trajs)
// {
//     for (auto &msg_traj : msg.person_trajectories) {
//         PassengerFlow::PersonTrajectory traj;
//         convert_msg_to_traj(msg_traj, traj);
//         trajs.push_back(traj);
//     }
// }


void convert_event_to_msg(const PassengerFlow::TrajectoryEvent &event, psg_private_msgs::msg::TrajectoryEvent &msg)
{
    msg.event_type = event.m_event_type;
    msg.start_time = event.m_start_time;
    msg.end_time = event.m_end_time;
    msg.track_id = event.m_person_id;
    msg.event_zone_name = event.m_event_zone->get_name();
    msg.event_pattern = event.m_event_pattern;
    msg.matched_trajectory = event.m_matched_trajectory;
    geometry_msgs::msg::Vector3 speed_2;
    speed_2.x = event.m_speed.x;
    speed_2.y = event.m_speed.y;
    speed_2.z = 0;
    msg.speed_2 = speed_2;
}


// void convert_events_to_msg(const std::vector<PassengerFlow::TrajectoryEvent> &events, psg_private_msgs::msg::TrajectoryEvents &msg)
// {
//     for (auto &event : events) {
//         psg_private_msgs::msg::TrajectoryEvent msg_traj_event;
//         convert_event_to_msg(event, msg_traj_event);
//         msg.trajectory_events.push_back(msg_traj_event);
//     }
// }

void convert_msg_to_keypoints(const redoxi_public_msgs::msg::Keypoints &msg, std::map<PassengerFlow::KeyPointSemanticType, PassengerFlow::Keypoint> &keypoints)
{
    for (size_t i = 0; i < msg.confidence.size(); i++) {
        PassengerFlow::KeyPointSemanticType pt_type = msg.semantic_type[i];
        PassengerFlow::Keypoint kpt;
        kpt.m_confidence = msg.confidence[i];
        auto &pt = msg.keypoints_2[i];
        kpt.m_point.x = pt.x;
        kpt.m_point.y = pt.y;
        keypoints[pt_type] = kpt;
    }
}
} // namespace FlowRos2Pipeline
