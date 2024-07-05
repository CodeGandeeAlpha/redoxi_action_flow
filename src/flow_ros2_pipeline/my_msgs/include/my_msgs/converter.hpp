#ifndef CONVERTER_HPP
#define CONVERTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "passenger_flow_header.h"
#include "my_msgs/msg/person.hpp"
#include "my_msgs/msg/person_trajectories.hpp"
#include "my_msgs/msg/trajectory_events.hpp"
#include "my_msgs/msg/persons.hpp"
#include "my_msgs/msg/poses.hpp"

namespace FlowRos2Pipeline {
    void convert_detection_to_msg(PassengerFlow::Detection& det, my_msgs::msg::Detection& msg);
    void convert_msg_to_detection(const my_msgs::msg::Detection& msg, PassengerFlow::DetectionPtr& det);

    void convert_person_to_msg(const PassengerFlow::PersonPtr& person, my_msgs::msg::Person& msg);
    void convert_msg_to_person(const my_msgs::msg::Person& msg, PassengerFlow::PersonPtr& person);

    void convert_persons_to_msg(const std::vector<PassengerFlow::PersonPtr>& persons, my_msgs::msg::Persons& msg);
    void convert_msg_to_persons(const my_msgs::msg::Persons& msg, std::vector<PassengerFlow::PersonPtr>& persons);

    void convert_poses_to_msg(const std::vector<PassengerFlow::PersonPtr>& poses, my_msgs::msg::Poses& msg);
    void convert_msg_to_poses(const my_msgs::msg::Poses& msg, std::vector<PassengerFlow::PersonPtr>& poses);

    void convert_traj_to_msg(const PassengerFlow::PersonTrajectory& traj, my_msgs::msg::PersonTrajectory& msg);
    void convert_msg_to_traj(const my_msgs::msg::PersonTrajectory& msg, PassengerFlow::PersonTrajectory& traj);

    void convert_trajs_to_msg(const std::vector<PassengerFlow::PersonTrajectory>& trajs, my_msgs::msg::PersonTrajectories& msg);
    void convert_msg_to_trajs(const my_msgs::msg::PersonTrajectories& msg, std::vector<PassengerFlow::PersonTrajectory>& trajs);

    void convert_event_to_msg(const PassengerFlow::TrajectoryEvent& event, my_msgs::msg::TrajectoryEvent& msg);
    void convert_msg_to_event(const my_msgs::msg::TrajectoryEvent& msg, PassengerFlow::TrajectoryEvent& event);

    void convert_events_to_msg(const std::vector<PassengerFlow::TrajectoryEvent>& events, my_msgs::msg::TrajectoryEvents& msg);
    void convert_msg_to_events(const my_msgs::msg::TrajectoryEvents& msg, std::vector<PassengerFlow::TrajectoryEvent>& events);
}

#endif // CONVERTER_HPP