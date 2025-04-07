#pragma once

#include <redoxi_common_cpp/common_concepts.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>

namespace redoxi_works
{

namespace input_port_types
{

using _SampleAction = redoxi_public_msgs::action::ProcessFrame;
using _SampleTimeUnit = std::chrono::microseconds;

/**
 * @brief Concept to define the requirements for a type to be used as source data in an action server.
 *
 * This concept ensures that a type meets the necessary requirements to be used as source data
 * in an action server. The constraints are as follows:
 *
 * - The type is copyable and default constructible to ensure it can be easily managed and instantiated.
 * - It defines an ActionType_t that satisfies the RosActionConcept, representing the action associated
 *   with the source data.
 * - It defines a Goal_t that matches the Goal type within the ActionType_t.
 * - It defines a GoalHandle_t that matches the ServerGoalHandle type for the ActionType_t.
 * - It defines a GoalHandlePromise_t that matches a promise of a shared pointer to a GoalHandle_t.
 * - It provides methods to get the goal UUID, the goal itself, the goal handle future, and the goal handle promise.
 */
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

//! Dummy class to satisfy the ReceiveSourceDataConcept
class _SampleReceiveSourceData
{
  public:
    using ActionType_t = _SampleAction;
    using Goal_t = typename ActionType_t::Goal;
    using GoalHandle_t = rclcpp_action::ServerGoalHandle<ActionType_t>;
    using GoalHandlePromise_t = std::promise<std::shared_ptr<GoalHandle_t>>;

    //! Get goal uuid
    rclcpp_action::GoalUUID get_goal_uuid() const
    {
        return rclcpp_action::GoalUUID{};
    }

    //! Get goal
    const Goal_t *get_goal() const
    {
        return nullptr;
    }

    //! Get goal handle future
    std::shared_future<std::shared_ptr<GoalHandle_t>> get_goal_handle_future()
    {
        return std::shared_future<std::shared_ptr<GoalHandle_t>>{};
    }

    //! Get goal handle promise
    std::shared_ptr<GoalHandlePromise_t> get_goal_handle_promise()
    {
        return std::make_shared<GoalHandlePromise_t>();
    }
};
static_assert(ReceiveSourceDataConcept<_SampleReceiveSourceData>);

/**
 * @brief Concept to define the specification for the async action input port.
 *
 * This concept ensures that the type T adheres to the required interface
 * for an async action input port specification. It checks for the presence
 * of necessary types and methods that are expected to be implemented by
 * any class conforming to this concept.
 *
 * Constraints:
 * - The type T must be capable of being copied, meaning it supports copy operations.
 * - The type T must be able to be constructed without any arguments, ensuring it has a default constructor.
 * - T must define a type `TimeUnit_t` that represents a duration of time.
 * - T must provide a method to retrieve the buffer capacity, which indicates the number of buffer requests.
 * - T must provide a method to retrieve the action name, which identifies the action being performed.
 * - T must provide a method to retrieve the goal result expiration time, which specifies how long a goal result is valid.
 */
template <typename T>
concept InitConfigConcept = requires(T t)
{
    requires std::copyable<T>;
    requires std::is_default_constructible_v<T>;

    //! The time unit type
    typename T::TimeUnit_t;
    requires TimeDurationConcept<typename T::TimeUnit_t>;

    //! The action type
    typename T::ActionType_t;
    requires RosActionConcept<typename T::ActionType_t>;

    //! Must have method to get buffer capacity
    {
        std::declval<const T &>().get_buffer_capacity()
        } -> std::same_as<int64_t>;

    //! Must have method to get action name
    {
        std::declval<const T &>().get_action_name()
        } -> std::same_as<const std::string &>;

    //! Must have method to get goal result expire time
    {
        std::declval<const T &>().get_goal_result_expire_time()
        } -> std::same_as<typename T::TimeUnit_t>;
};

//! A sample class that satisfies the InitConfigConcept
class _SampleInitConfig
{
  public:
    using TimeUnit_t = _SampleTimeUnit;
    using ActionType_t = _SampleAction;
    //! Get number of buffer requests
    //! If the value is not positive, it means the queue is unbounded
    int64_t get_buffer_capacity() const
    {
        return 0;
    }

    //! Get the name of the action
    const std::string &get_action_name() const
    {
        static std::string action_name = "default_action";
        return action_name;
    }

    //! Get the goal result expire time
    TimeUnit_t get_goal_result_expire_time() const
    {
        return TimeUnit_t(0);
    }
};
static_assert(InitConfigConcept<_SampleInitConfig>);

/**
 * @brief Concept that includes all the types required by AsyncActionInputPort.
 *
 * This concept ensures the presence of the following types:
 * - ActionType_t: The type of the action.
 * - ActionGoal_t: The goal type of the action, must match ActionType_t::Goal.
 * - ActionDataTrait_t: The data trait type for the action, must match ActionType_t.
 * - TimeUnit_t: The time unit type, must satisfy TimeDurationConcept.
 * - ReceiveSourceData_t: The type for received source data, must match ActionType_t.
 * - InitConfig_t: The initialization configuration type, must match TimeUnit_t.
 */
template <typename T>
concept AsyncActionInputPortSpecConcept = requires
{
    typename T::ActionType_t;
    requires RosActionConcept<typename T::ActionType_t>;

    typename T::ActionGoal_t;
    requires std::same_as<typename T::ActionGoal_t, typename T::ActionType_t::Goal>;

    typename T::ActionDataTrait_t;
    requires ActionDataTraitConcept<typename T::ActionDataTrait_t>;
    requires std::same_as<typename T::ActionDataTrait_t::ActionType_t, typename T::ActionType_t>;

    typename T::TimeUnit_t;
    requires TimeDurationConcept<typename T::TimeUnit_t>;

    typename T::ReceiveSourceData_t;
    requires ReceiveSourceDataConcept<typename T::ReceiveSourceData_t>;
    requires std::same_as<typename T::ReceiveSourceData_t::ActionType_t, typename T::ActionType_t>;

    typename T::InitConfig_t;
    requires InitConfigConcept<typename T::InitConfig_t>;
    requires std::same_as<typename T::InitConfig_t::TimeUnit_t, typename T::TimeUnit_t>;
};


class _SampleAsyncActionInputPortSpec
{
  public:
    using ActionType_t = _SampleAction;
    using ActionGoal_t = typename ActionType_t::Goal;
    using ActionDataTrait_t = NoneActionDataTrait<ActionType_t>;
    using TimeUnit_t = _SampleTimeUnit;
    using ReceiveSourceData_t = _SampleReceiveSourceData;
    using InitConfig_t = _SampleInitConfig;
};
static_assert(AsyncActionInputPortSpecConcept<_SampleAsyncActionInputPortSpec>);

} // namespace input_port_types

} // namespace redoxi_works
