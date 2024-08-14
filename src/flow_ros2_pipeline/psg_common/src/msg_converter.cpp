#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <psg_common/msg_converter.hpp>
#include <psg_private_msgs/msg/detail/person__struct.hpp>
#include <psg_private_msgs/msg/detail/person_trajectory__struct.hpp>
#include <psg_private_msgs/msg/detail/trajectory_event__struct.hpp>

namespace FlowRos2Pipeline
{
    void convert_detection_to_msg(PassengerFlow::Detection& det, psg_public_msgs::msg::Detection& msg)
    {
        auto bbox = det.get_bbox();
        msg.bbox.x = bbox.x;
        msg.bbox.y = bbox.y;
        msg.bbox.width = bbox.width;
        msg.bbox.height = bbox.height;
        msg.category = det.get_type();
        msg.confidence = det.get_confidence();
        msg.frame.frame_num = det.get_frame_number();

        if (det.get_type() == RedoxiTrack::DetectionTypes::PersonBody) {
            RedoxiTrack::fVECTOR feature;
            det.get_feature(feature);
            std::vector<double> vec(feature.data(), feature.data()+feature.rows()*feature.cols());
            msg.feature = vec;
        }
    }


    void convert_msg_to_detection(const psg_public_msgs::msg::Detection& msg, PassengerFlow::DetectionPtr& det) {
        auto bbox = PassengerFlow::BBOX(msg.bbox.x, msg.bbox.y, msg.bbox.width, msg.bbox.height);
        det->set_bbox(bbox);
        det->set_confidence(msg.confidence);

        if (msg.category == RedoxiTrack::DetectionTypes::PersonBody) {
            RedoxiTrack::fVECTOR single_feature;
            single_feature.resize(128, 1);
            for (int j = 0; j < msg.feature.size(); j++) {
                single_feature(j, 0) = msg.feature[j];
            }
            det->set_type(RedoxiTrack::DetectionTypes::PersonBody);
            det->set_feature(single_feature);
        }
        else if (msg.category == RedoxiTrack::DetectionTypes::PersonFace) {
            det->set_type(RedoxiTrack::DetectionTypes::PersonFace);
        }
        else {
            det->set_type(RedoxiTrack::DetectionTypes::PersonHead);
        }

        det->set_frame_number(msg.frame.frame_num);
        det->set_detected_by_camera(msg.is_detected_by_camera);
    }


    void convert_person_to_msg(const PassengerFlow::PersonPtr& person, psg_private_msgs::msg::Person& msg) {
        msg.body.category = -1;
        msg.face.category = -1;
        msg.head.category = -1;
        // msg.has_pose = false;
        // msg.has_foot_position = false;
        // msg.has_head_position = false;

        if (person->body()) {
            psg_public_msgs::msg::Detection msg_body;
            auto* body_ptr = RedoxiTrack::dyncast_with_check<PassengerFlow::Detection>(person->body().get());
            convert_detection_to_msg(*body_ptr, msg_body);
            msg.body = msg_body;
        }
        if (person->head()) {
            psg_public_msgs::msg::Detection msg_head;
            auto* head_ptr = RedoxiTrack::dyncast_with_check<PassengerFlow::Detection>(person->head().get());
            convert_detection_to_msg(*head_ptr, msg_head);
            msg.head = msg_head;
        }
        if (person->face()) {
            psg_public_msgs::msg::Detection msg_face;
            auto* face_ptr = RedoxiTrack::dyncast_with_check<PassengerFlow::Detection>(person->face().get());
            convert_detection_to_msg(*face_ptr, msg_face);
            msg.face = msg_face;
        }

        auto person_pose = person->get_keypoints();
        if (person_pose.size() > 0) {
            psg_public_msgs::msg::BodyPose msg_pose;
            for (auto& kp : person_pose) {
                auto kp_type = kp.first;
                msg_pose.semantic_type.push_back(kp_type);
                msg_pose.confidence.push_back(kp.second.m_confidence);
                // msg_pose.x.push_back(kp.second.m_point.x);
                // msg_pose.y.push_back(kp.second.m_point.y);
                geometry_msgs::msg::Point pt;
                pt.x = kp.second.m_point.x;
                pt.y = kp.second.m_point.y;
                pt.z = 0;
                msg_pose.keypoints_2.push_back(pt);
            }
            msg.pose = msg_pose;
        }

        msg.track_id = person->get_person_id();
        msg.frame.frame_num = person->get_frame_number();
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


    void convert_msg_to_person(const psg_private_msgs::msg::Person& msg, PassengerFlow::PersonPtr& person) {
        if(msg.body.category != -1) {
            PassengerFlow::DetectionPtr one_det = std::make_shared<PassengerFlow::Detection>();
            convert_msg_to_detection(msg.body, one_det);
            person->set_body(one_det);
        }

        if(msg.head.category != -1) {
            PassengerFlow::DetectionPtr one_det = std::make_shared<PassengerFlow::Detection>();
            convert_msg_to_detection(msg.head, one_det);
            person->set_head(one_det);
        }

        if(msg.face.category != -1) {
            PassengerFlow::DetectionPtr one_det = std::make_shared<PassengerFlow::Detection>();
            convert_msg_to_detection(msg.face, one_det);
            person->set_face(one_det);
        }

        if(msg.pose.keypoints_2.size() > 0) {
            std::map<PassengerFlow::KeyPointSemanticType, PassengerFlow::Keypoint> pose;
            for(int i = 0; i < msg.pose.confidence.size(); i++) {
                PassengerFlow::KeyPointSemanticType pt_type = msg.pose.semantic_type[i];
                PassengerFlow::Keypoint kpt;
                kpt.m_confidence = msg.pose.confidence[i];
                auto& pt = msg.pose.keypoints_2[i];
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

        if(msg.foot_position_3.x != 0 || msg.foot_position_3.y != 0 || msg.foot_position_3.z != 0) {
            person->set_foot_position({float(msg.foot_position_3.x), float(msg.foot_position_3.y), float(msg.foot_position_3.z)});
        }

        if(msg.head_position_3.x != 0 || msg.head_position_3.y != 0 || msg.head_position_3.z != 0) {
            person->set_head_position({float(msg.head_position_3.x), float(msg.head_position_3.y), float(msg.head_position_3.z)});
        }

        person->set_person_id(msg.track_id);
        person->set_frame_number(msg.frame.frame_num);
    }


    void convert_persons_to_msg(const std::vector<PassengerFlow::PersonPtr>& persons, psg_private_msgs::msg::Persons& msg) {
        for (auto& person: persons) {
            psg_private_msgs::msg::Person msg_person;
            convert_person_to_msg(person, msg_person);
            msg.persons.push_back(msg_person);
            msg.frame = msg_person.frame;
        }
    }


    void convert_msg_to_persons(const psg_private_msgs::msg::Persons& msg, std::vector<PassengerFlow::PersonPtr>& persons) {
        for (auto& msg_person : msg.persons) {
            auto person_ptr = std::make_shared<PassengerFlow::Person>();
            convert_msg_to_person(msg_person, person_ptr);
            persons.push_back(person_ptr);
        }
    }


    void convert_traj_to_msg(const PassengerFlow::PersonTrajectory& traj, psg_private_msgs::msg::PersonTrajectory& msg) {
        int person_id = traj.get_person_id();
        msg.track_id = person_id;

        for(auto& person: traj.m_person_list) {
            psg_private_msgs::msg::Person msg_person;
            convert_person_to_msg(person, msg_person);
            msg.persons.push_back(msg_person);
        }
    }


    void convert_msg_to_traj(const psg_private_msgs::msg::PersonTrajectory& msg, PassengerFlow::PersonTrajectory& traj) {
        for(auto& msg_person: msg.persons) {
            PassengerFlow::PersonPtr person = std::make_shared<PassengerFlow::Person> ();
            convert_msg_to_person(msg_person, person);
            person->set_person_id(msg.track_id);
            traj.insert(person);
        }
    }


    void convert_trajs_to_msg(const std::vector<PassengerFlow::PersonTrajectory>& trajs, psg_private_msgs::msg::PersonTrajectories& msg) {
        for (auto& traj: trajs) {
            psg_private_msgs::msg::PersonTrajectory msg_traj;
            convert_traj_to_msg(traj, msg_traj);
            msg.person_trajectories.push_back(msg_traj);
        }
    }


    void convert_msg_to_trajs(const psg_private_msgs::msg::PersonTrajectories& msg, std::vector<PassengerFlow::PersonTrajectory>& trajs) {
        for (auto& msg_traj : msg.person_trajectories) {
            PassengerFlow::PersonTrajectory traj;
            convert_msg_to_traj(msg_traj, traj);
            trajs.push_back(traj);
        }
    }


    void convert_event_to_msg(const PassengerFlow::TrajectoryEvent& event, psg_private_msgs::msg::TrajectoryEvent& msg) {
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


    void convert_events_to_msg(const std::vector<PassengerFlow::TrajectoryEvent>& events, psg_private_msgs::msg::TrajectoryEvents& msg) {
        for (auto& event : events) {
            psg_private_msgs::msg::TrajectoryEvent msg_traj_event;
            convert_event_to_msg(event, msg_traj_event);
            msg.trajectory_events.push_back(msg_traj_event);
        }
    }
}