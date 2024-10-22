#pragma once

#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <algorithm>

#include <unique_identifier_msgs/msg/uuid.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>


namespace redoxi_works
{

//! Convert a ROS 2 UUID message to a boost UUID
inline boost::uuids::uuid uuid_from_ros_msg(const unique_identifier_msgs::msg::UUID &msg)
{
    boost::uuids::uuid uuid;
    std::copy(msg.uuid.begin(), msg.uuid.end(), uuid.begin());
    return uuid;
}

//! Convert a boost UUID to a ROS 2 UUID message
inline unique_identifier_msgs::msg::UUID uuid_to_ros_msg(const boost::uuids::uuid &uuid)
{
    unique_identifier_msgs::msg::UUID msg;
    std::copy(uuid.begin(), uuid.end(), msg.uuid.begin());
    return msg;
}

} // namespace redoxi_works
