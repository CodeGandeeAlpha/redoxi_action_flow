#pragma once

#include <PassengerFlow/PassengerFlow.h>
#include <psg_public_msgs/msg/detections.hpp>
#include <psg_private_msgs/msg/persons.hpp>
#include <psg_private_msgs/msg/trajectory_events.hpp>
#include <psg_private_msgs/msg/person_trajectories.hpp>

namespace FlowRos2Pipeline
{
    void convert_detection_to_msg(PassengerFlow::Detection& det, psg_public_msgs::msg::Detection& msg);
    void convert_msg_to_detection(const psg_public_msgs::msg::Detection& msg, PassengerFlow::DetectionPtr& det);

    void convert_person_to_msg(const PassengerFlow::PersonPtr& person, psg_private_msgs::msg::Person& msg);
    void convert_msg_to_person(const psg_private_msgs::msg::Person& msg, PassengerFlow::PersonPtr& person);

    void convert_persons_to_msg(const std::vector<PassengerFlow::PersonPtr>& persons, psg_private_msgs::msg::Persons& msg);
    void convert_msg_to_persons(const psg_private_msgs::msg::Persons& msg, std::vector<PassengerFlow::PersonPtr>& persons);

    // void convert_poses_to_msg(const std::vector<PassengerFlow::PersonPtr>& poses, psg_private_msgs::Poses& msg);
    // void convert_msg_to_poses(const psg_private_msgs::Poses& msg, std::vector<PassengerFlow::PersonPtr>& poses);

    void convert_traj_to_msg(const PassengerFlow::PersonTrajectory& traj, psg_private_msgs::msg::PersonTrajectory& msg);
    void convert_msg_to_traj(const psg_private_msgs::msg::PersonTrajectory& msg, PassengerFlow::PersonTrajectory& traj);

    void convert_trajs_to_msg(const std::vector<PassengerFlow::PersonTrajectory>& trajs, psg_private_msgs::msg::PersonTrajectories& msg);
    void convert_msg_to_trajs(const psg_private_msgs::msg::PersonTrajectories& msg, std::vector<PassengerFlow::PersonTrajectory>& trajs);

    void convert_event_to_msg(const PassengerFlow::TrajectoryEvent& event, psg_private_msgs::msg::TrajectoryEvent& msg);
    // void convert_msg_to_event(const psg_private_msgs::msg::TrajectoryEvent& msg, PassengerFlow::TrajectoryEvent& event);

    void convert_events_to_msg(const std::vector<PassengerFlow::TrajectoryEvent>& events, psg_private_msgs::msg::TrajectoryEvents& msg);
    // void convert_msg_to_events(const psg_private_msgs::msg::TrajectoryEvents& msg, std::vector<PassengerFlow::TrajectoryEvent>& events);
}
