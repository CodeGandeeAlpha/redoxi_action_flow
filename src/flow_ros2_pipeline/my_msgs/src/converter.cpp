#include "my_msgs/converter.hpp"

namespace FlowRosPipeline {
    void convert_detection_to_msg(PassengerFlow::Detection &det, my_msgs::msg::Detection& msg) {
        auto bbox = det.get_bbox();
        msg.x = bbox.x;
        msg.y = bbox.y;
        msg.w = bbox.width;
        msg.h = bbox.height;
        msg.category = det.get_type();
        msg.is_detected_by_camera = det.detected_by_camera();
        msg.confidence = det.get_confidence();
        msg.frame_num = det.get_frame_number();

        if (det.get_type() == IntellifTracking::DetectioncTypes::PersonBody) {
            IntellifTracking::fVECTOR feature;
            det.get_feature(feature);
            std::vector<float> vec(feature.data(), feature.data()+feature.rows()*feature.cols());
            msg.idfeat = vec;
        }
    }
    void convert_msg_to_detection(const my_msgs::msg::Detection& msg, PassengerFlow::DetectionPtr& det) {
        auto bbox = PassengerFlow::BBOX(msg.x, msg.y, msg.w, msg.h);
        det->set_bbox(bbox);
        det->set_confidence(msg.confidence);

        if (msg.category == IntellifTracking::DetectionTypes::PersonBody) {
            IntellifTracking::fVECTOR single_feature;
            single_feature.resize(128, 1);
            for (int j = 0; j < msg.idfeat.size(); j++) {
                single_feature(j, 0) = msg.idfeat[j];
            }
            det->set_type(IntellifTracking::DetectionTypes::PersonBody);
            det->set_feature(single_feature);
        }
        else if (msg.category == IntellifTracking::DetectionTypes::PersonFace) {
            det->set_type(IntellifTracking::DetectionTypes::PersonFace);
        }
        else {
            det->set_type(IntellifTracking::DetectionTypes::PersonHead);
        }

        det->set_frame_number(msg.frame_num);
        det->set_detected_by_camera(msg.is_detected_by_camera);
    }

    void convert_person_to_msg(const PassengerFlow::PersonPtr& person, my_msgs::msg::Person& msg) {
        msg.has_body = false;
        msg.has_face = false;
        msg.has_head = false;
        msg.has_pose = false;
        msg.has_foot_position = false;
        msg.has_head_position = false;

        if (person->body()) {
            my_msgs::msg::Detection msg_body;
            auto* body_ptr = IntellifTracking::dyncast_with_check<PassengerFlow::Detection>(person->body().get());
            convert_detection_to_msg(*body_ptr, msg_body);
            msg.has_body = true;
            msg.person_body = msg_body;
        }
        if (person->head()) {
            my_msgs::msg::Detection msg_head;
            auto* head_ptr = IntellifTracking::dyncast_with_check<PassengerFlow::Detection>(person->head().get());
            convert_detection_to_msg(*head_ptr, msg_head);
            msg.has_head = true;
            msg.person_head = msg_head;
        }
        if (person->face()) {
            my_msgs::msg::Detection msg_face;
            auto* face_ptr = IntellifTracking::dyncast_with_check<PassengerFlow::Detection>(person->face().get());
            convert_detection_to_msg(*face_ptr, msg_face);
            msg.has_face = true;
            msg.person_face = msg_face;
        }

        auto person_pose = person->get_keypoints();
        if (person_pose.size() > 0) {
            my_msgs::msg::Pose msg_pose;
            for (auto& kp : person_pose) {
                auto kp_type = kp.first;
                msg_pose.semantic_type.push_back(kp_type);
                msg_pose.confidence.push_back(kp.second.m_confidence);
                msg_pose.x.push_back(kp.second.m_point.x);
                msg_pose.y.push_back(kp.second.m_point.y);
            }
            msg.has_pose = true;
            msg.person_pose = msg_pose;
        }

        msg.id = person->get_id();
        msg.frame_number = person->get_frame_number();
        msg.body_height = person->get_body_height().m_body_height;
        msg.body_height_conf = person->get_body_height().m_body_height_conf;
        PassengerFlow::POINT3 position;
        bool position_valid;
        person->get_foot_position(&position, &position_valid);
        if (position_valid) {
            msg.has_foot_position = true;
            msg.foot_position = {position.x, position.y, position.z};
        }
        person->get_head_position(&position, &position_valid);
        if (position_valid) {
            msg.has_head_position = true;
            msg.head_position = {position.x, position.y, position.z};
        }
    }
    void convert_msg_to_person(const my_msgs::msg::Person& msg, PassengerFlow::PersonPtr& person) {
        if(msg.has_body) {
            PassengerFlow::DetectionPtr one_det = std::make_shared<PassengerFlow::Detection>();
            convert_msg_to_detection(msg.person_body, one_det);
            person->set_body(one_det);
        }

        if(msg.has_head) {
            PassengerFlow::DetectionPtr one_det = std::make_shared<PassengerFlow::Detection>();
            convert_msg_to_detection(msg.person_head, one_det);
            person->set_head(one_det);
        }

        if(msg.has_face) {
            PassengerFlow::DetectionPtr one_det = std::make_shared<PassengerFlow::Detection>();
            convert_msg_to_detection(msg.person_face, one_det);
            person->set_face(one_det);
        }

        if(msg.has_pose) {
            std::map<PassengerFlow::KeyPointSemanticType, PassengerFlow::Keypoint> pose;
            for(int i = 0; i < msg.person_pose.confidence.size(); i++) {
                PassengerFlow::KeyPointSemanticType pt_type = msg.person_pose.semantic_type[i];
                PassengerFlow::Keypoint kpt;
                kpt.m_confidence = msg.person_pose.confidence[i];
                kpt.m_point.x = msg.person_pose.x[i];
                kpt.m_point.y = msg.person_pose.y[i];
                kpt.m_semantic_id = pt_type;
                pose[pt_type] = kpt;
            }
            person->set_keypoints(pose);
        }

        PassengerFlow::BodyHeight height;
        height.m_body_height = msg.body_height;
        height.m_body_height_conf = msg.body_height_conf;
        person->set_body_height(height);

        if(msg.has_foot_position) {
            person->set_foot_position({msg.foot_position[0], msg.foot_position[1], msg.foot_position[2]});
        }

        if(msg.has_head_position) {
            person->set_head_position({msg.head_position[0], msg.head_position[1], msg.head_position[2]});
        }

        person->set_person_id(msg.id);
        person->set_frame_number(msg.frame_number);
    }

    void convert_persons_to_msg(const std::vector<PassengerFlow::PersonPtr>& persons, my_msgs::msg::Persons& msg) {
        for (auto& person: persons) {
            my_msgs::msg::Person msg_person;
            msg_person.header = msg.header;
            convert_person_to_msg(person, msg_person);
            msg.persons.push_back(msg_person);
        }
    }
    void convert_msg_to_persons(const my_msgs::msg::Persons& msg, std::vector<PassengerFlow::PersonPtr>& persons) {
        for (auto& msg_person : msg.persons) {
            auto person_ptr = std::make_shared<PassengerFlow::Person>();
            convert_msg_to_person(msg_person, person_ptr);
            persons.push_back(person_ptr);
        }
    }

    void convert_poses_to_msg(const std::vector<PassengerFlow::PersonPtr>& poses, my_msgs::msg::Poses& msg) {
        for (auto& person: poses) {
            my_msgs::msg::Person msg_person;
            msg_person.header = msg.header;
            convert_person_to_msg(person, msg_person);
            msg.persons.push_back(msg_person);
        }
    }
    void convert_msg_to_poses(const my_msgs::msg::Poses& msg, std::vector<PassengerFlow::PersonPtr>& poses) {
        for (auto& msg_person : msg.persons) {
            auto person_ptr = std::make_shared<PassengerFlow::Person>();
            convert_msg_to_person(msg_person, person_ptr);
            poses.push_back(person_ptr);
        }
    }


    void convert_traj_to_msg(const PassengerFlow::PersonTrajectory& traj, my_msgs::msg::PersonTrajectory& msg) {
        int person_id = traj.get_person_id();
        msg.person_id = person_id;

        for(auto& person: traj.m_person_list) {
            my_msgs::msg::Person msg_person;
            msg_person.header = msg.header;
            convert_person_to_msg(person, msg_person);
            msg.persons.push_back(msg_person);
        }
    }
    void convert_msg_to_traj(const my_msgs::msg::PersonTrajectory& msg, PassengerFlow::PersonTrajectory& traj) {
        for(auto& msg_person: msg.persons) {
            PassengerFlow::PersonPtr person = std::make_shared<PassengerFlow::Person> ();
            convert_msg_to_person(msg_person, person);
            person->set_person_id(msg.person_id);
            traj.insert(person);
        }
    }

    void convert_trajs_to_msg(const std::vector<PassengerFlow::PersonTrajectory>& trajs, my_msgs::msg::PersonTrajectories& msg) {
        for (auto& traj: trajs) {
            my_msgs::msg::PersonTrajectory msg_traj;
            msg_traj.header = msg.header;
            convert_traj_to_msg(traj, msg_traj);
            msg.person_trajectories.push_back(msg_traj);
        }
    }
    void convert_msg_to_trajs(const my_msgs::msg::PersonTrajectories& msg, std::vector<PassengerFlow::PersonTrajectory>& trajs) {
        for (auto& msg_traj : msg.person_trajectories) {
            PassengerFlow::PersonTrajectory traj;
            convert_msg_to_traj(msg_traj, traj);
            trajs.push_back(traj);
        }
    }

    void convert_event_to_msg(const PassengerFlow::TrajectoryEvent& event, my_msgs::msg::TrajectoryEvent& msg) {
        msg.event_type = event.m_event_type;
        msg.start_time = event.m_start_time;
        msg.end_time = event.m_end_time;
        msg.person_id = event.m_person_id;
        msg.event_zone_name = event.m_event_zone->get_name();
        msg.event_pattern = event.m_event_pattern;
        msg.matched_trajectory = event.m_matched_trajectory;
        msg.speed = {event.m_speed.x, event.m_speed.y};
    }
    // void convert_msg_to_event(const my_msgs::msg::TrajectoryEvent& msg, PassengerFlow::TrajectoryEvent& event) {
    //     event.m_event_type = msg.event_type;
    //     event.m_start_time = msg.start_time;
    //     event.m_end_time = msg.end_time;
    //     event.m_person_id = msg.person_id;
    //     event.m_event_zone = std::make_shared<PassengerFlow::EventZone>();
    //     event.m_event_zone->set_name(msg.event_zone_name);
    //     event.m_event_pattern = msg.event_pattern;
    //     event.m_matched_trajectory = msg.matched_trajectory;
    //     event.m_speed.x = msg.speed[0];
    //     event.m_speed.y = msg.speed[1];
    // }

    void convert_events_to_msg(const std::vector<PassengerFlow::TrajectoryEvent>& events, my_msgs::msg::TrajectoryEvents& msg) {
        for (auto& event : events) {
            my_msgs::msg::TrajectoryEvent msg_traj_event;
            msg_traj_event.header = msg.header;
            convert_event_to_msg(event, msg_traj_event);
            msg.trajectory_events.push_back(msg_traj_event);
        }
    }
    // void convert_msg_to_events(const my_msgs::msg::TrajectoryEvents& msg, std::vector<PassengerFlow::TrajectoryEvent>& events) {
    //     for (auto& msg_traj_event : msg.trajectory_events) {
    //         PassengerFlow::TrajectoryEvent event;
    //         convert_msg_to_event(msg_traj_event, event);
    //         events.push_back(event);
    //     }
    // }

}