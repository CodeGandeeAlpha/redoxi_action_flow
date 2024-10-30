#pragma once

#include <redoxi_common_nodes/common_concepts.hpp>
#include <redoxi_common_cpp/ros_utils/message_conversion.hpp>
#include <redoxi_public_msgs/msg/control.hpp>

namespace redoxi_works
{
//! Concept to check if a type is a Redoxi action, which is used by most of this library
template <typename T>
concept RedoxiActionConcept = requires(T t)
{
    // must be a ROS action
    requires RosActionConcept<T>;

    // the goal must have a control signal message
    {
        std::declval<typename T::Goal>().x_control
        } -> std::convertible_to<redoxi_public_msgs::msg::Control>;

    // the goal must have a UUID
    {
        std::declval<typename T::Goal>().x_uid
        } -> std::convertible_to<unique_identifier_msgs::msg::UUID>;
};

//! Default implementation of ActionDataTraitConcept
//! For use with control.msg in the goal
template <RedoxiActionConcept ActionType>
struct RedoxiActionDataTrait {
    using ActionType_t = ActionType;
    using Goal_t = typename ActionType_t::Goal;
    using Result_t = typename ActionType_t::Result;
    using Feedback_t = typename ActionType_t::Feedback;
    using ControlSignalMessage_t = redoxi_public_msgs::msg::Control;

    RedoxiActionDataTrait()
    {
        static_assert(ActionDataTraitConcept<RedoxiActionDataTrait>, "RedoxiActionDataTrait must satisfy ActionDataTraitConcept");
    }

    static ControlSignalCode get_control_signal_code(const Goal_t &goal)
    {
        return ControlSignalCode(goal.x_control.code);
    }

    static void mark_with_control_signal(Goal_t &goal, ControlSignalCode code)
    {
        goal.x_control.code = int64_t(code);
    }

    static boost::uuids::uuid get_uuid(const Goal_t &goal)
    {
        return to_boost_uuid(goal.x_uid);
    }

    static void set_uuid(Goal_t &goal, const boost::uuids::uuid &uuid)
    {
        goal.x_uid = to_ros_goal_uuid(uuid);
    }
};

} // namespace redoxi_works