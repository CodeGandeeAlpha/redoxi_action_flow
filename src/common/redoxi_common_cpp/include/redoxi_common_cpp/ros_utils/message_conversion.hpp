#pragma once

#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <algorithm>

#include <unique_identifier_msgs/msg/uuid.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>


namespace redoxi_works
{

//! Convert a ROS 2 UUID message to a boost UUID
inline boost::uuids::uuid to_boost_uuid(const unique_identifier_msgs::msg::UUID &msg)
{
    boost::uuids::uuid uuid;
    std::copy(msg.uuid.begin(), msg.uuid.end(), uuid.begin());
    return uuid;
}

//! Convert a boost UUID to a ROS 2 UUID message
inline unique_identifier_msgs::msg::UUID to_ros_uuid_msg(const boost::uuids::uuid &uuid)
{
    unique_identifier_msgs::msg::UUID msg;
    std::copy(uuid.begin(), uuid.end(), msg.uuid.begin());
    return msg;
}

//! Convert a ROS 2 action goal UUID to a boost UUID
inline boost::uuids::uuid to_boost_uuid(const rclcpp_action::GoalUUID &goal_uuid)
{
    boost::uuids::uuid uuid;
    std::copy(goal_uuid.begin(), goal_uuid.end(), uuid.begin());
    return uuid;
}

//! Convert a boost UUID to a ROS 2 action goal UUID
inline rclcpp_action::GoalUUID to_ros_goal_uuid(const boost::uuids::uuid &uuid)
{
    rclcpp_action::GoalUUID goal_uuid;
    std::copy(uuid.begin(), uuid.end(), goal_uuid.begin());
    return goal_uuid;
}


} // namespace redoxi_works
