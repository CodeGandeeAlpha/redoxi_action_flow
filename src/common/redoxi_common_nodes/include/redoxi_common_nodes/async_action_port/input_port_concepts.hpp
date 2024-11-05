#pragma once

#include <redoxi_common_cpp/common_concepts.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace redoxi_works
{

namespace input_port_types
{

//! A data received by action server (the port)
template <typename T>
concept ReceiveSourceDataConcept = requires(T t)
{
    requires std::copyable<T>;
    requires std::is_default_constructible_v<T>;

    //! The source data comes from an action
    typename T::ActionType_t;
    requires RosActionConcept<typename T::ActionType_t>;

    //! The goal type must match the action goal type
    typename T::Goal_t;
    requires std::same_as<typename T::Goal_t, typename T::ActionType_t::Goal>;

    //! The goal handle type must match the action goal handle type
    typename T::GoalHandle_t;
    requires std::same_as<typename T::GoalHandle_t, rclcpp_action::ServerGoalHandle<typename T::ActionType_t>>;

    //! The goal promise type must match the action goal handle type
    typename T::GoalHandlePromise_t;
    requires std::same_as<typename T::GoalHandlePromise_t, std::promise<std::shared_ptr<typename T::GoalHandle_t>>>;

    //! Must have method to get goal uuid
    {
        std::declval<const T &>().get_goal_uuid()
        } -> std::same_as<rclcpp_action::GoalUUID>;

    //! Must have method to get goal
    {
        std::declval<const T &>().get_goal()
        } -> std::same_as<const typename T::Goal_t *>;

    //! Must have method to get goal handle future
    {
        t.get_goal_handle_future()
        } -> std::same_as<std::shared_future<std::shared_ptr<typename T::GoalHandle_t>>>;

    //! Must have method to get goal handle promise
    {
        t.get_goal_handle_promise()
        } -> std::same_as<std::shared_ptr<typename T::GoalHandlePromise_t>>;
};

} // namespace input_port_types

} // namespace redoxi_works
