#pragma once

#include <string>
#include <optional>
#include <concepts>
#include <utility>
#include <type_traits>
#include <chrono>
#include <limits>

#include <unique_identifier_msgs/msg/uuid.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp/node.hpp>
#include <boost/uuid/uuid.hpp>
#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works
{

//! Control signal codes used to indicate special actions in messages
//! @note If a message contains other messages that contain control signals,
//!       only the top level control message is used
enum class ControlSignalCode {
    Normal = 0,                                    //!< Normal signal, no special action needed
    Ping = 1,                                      //!< Ping signal - downstream should reply but not process data
    Flush = 2,                                     //!< Flush signal - finish processing previous data and process new data in clean state
    Reset = 3,                                     //!< Reset signal - reset to initial state as if previous data never existed
    Terminate = 4,                                 //!< Terminate signal - no more data will be sent after this
    Unknown = std::numeric_limits<int32_t>::max(), //!< Unknown signal, should be ignored
};

//! Concept to check if a type is ROS message
template <typename T>
concept RosMessageConcept = requires(T t)
{
    requires std::copyable<T>;
    requires std::is_copy_assignable_v<T>;
    requires std::is_default_constructible_v<T>;
};

//! Concept to check if a type is ROS action definition
template <typename T>
concept RosActionConcept = requires
{
    // copyable and default constructible
    requires std::copyable<T>;
    requires std::is_default_constructible_v<T>;

    //! Check Goal type exists and is accessible
    typename T::Goal;
    requires RosMessageConcept<typename T::Goal>;

    //! Check Result type exists and is accessible
    typename T::Result;
    requires RosMessageConcept<typename T::Result>;

    //! Check Feedback type exists and is accessible
    typename T::Feedback;
    requires RosMessageConcept<typename T::Feedback>;

    //! Check Impl struct exists and has required service/message types
    typename T::Impl::SendGoalService;
    typename T::Impl::GetResultService;
    typename T::Impl::FeedbackMessage;
    typename T::Impl::CancelGoalService;
    typename T::Impl::GoalStatusMessage;
};

//! Publisher concept
template <typename T>
concept RosPublisherConcept = requires(T pub)
{
    //! Publish a message, without additional text
    typename T::MessageType_t;
    requires RosMessageConcept<typename T::MessageType_t>;
    {
        pub.publish(std::declval<const typename T::MessageType_t &>())
        } -> std::same_as<int>;

    //! Publish a message with additional text
    {
        pub.publish(std::declval<const typename T::MessageType_t &>(),
                    std::declval<const std::string &>())
        } -> std::same_as<int>;
};

//! Concept to check if a type is std::chrono::duration
template <typename T>
concept TimeDurationConcept = requires
{
    requires std::is_same_v<T, std::chrono::duration<typename T::rep, typename T::period>>;
};

template <typename T>
concept ActionDataTraitConcept = requires(T t)
{
    typename T::ActionType_t;
    requires RosActionConcept<typename T::ActionType_t>;

    typename T::Goal_t;
    requires std::same_as<typename T::Goal_t, typename T::ActionType_t::Goal>;

    typename T::Result_t;
    requires std::same_as<typename T::Result_t, typename T::ActionType_t::Result>;

    typename T::Feedback_t;
    requires std::same_as<typename T::Feedback_t, typename T::ActionType_t::Feedback>;

    // get control signal code from goal
    {
        T::get_control_signal_code(std::declval<const typename T::Goal_t &>())
        } -> std::same_as<ControlSignalCode>;

    // mark a goal with a control signal
    {
        T::mark_with_control_signal(std::declval<typename T::Goal_t &>(), std::declval<ControlSignalCode>())
        } -> std::same_as<void>;

    // get uuid from goal
    {
        T::get_uuid(std::declval<const typename T::Goal_t &>())
        } -> std::same_as<boost::uuids::uuid>;

    // write uuid to goal
    {
        T::set_uuid(std::declval<typename T::Goal_t &>(), std::declval<boost::uuids::uuid>())
        } -> std::same_as<void>;
};

//! Interface for retry policy, if anything needs to retry, its configuration should be here
//! Concept for retry policy interface
template <typename T>
concept RetryPolicyConcept = requires(T t,
                                      std::optional<int64_t> retry_count,
                                      std::optional<typename T::DurationType_t> wait_time,
                                      bool use_fallback_if_not_set)
{
    typename T::DurationType_t;
    requires TimeDurationConcept<typename T::DurationType_t>;

    //! Must be default constructible
    requires std::is_default_constructible_v<T>;

    //! Must be copyable
    requires std::copyable<T>;

    //! Must have methods to get/set number of retries
    {
        std::declval<const T &>().get_number_of_retry(use_fallback_if_not_set)
        } -> std::same_as<std::optional<int64_t>>;
    {
        t.set_number_of_retry(retry_count)
        } -> std::same_as<void>;
    {
        std::declval<const T &>().get_fallback_number_of_retry()
        } -> std::same_as<int64_t>;

    //! Must have methods to get/set wait time between retries
    //! negative wait time means wait indefinitely, 0 means no wait
    {
        std::declval<const T &>().get_wait_time_between_retry(use_fallback_if_not_set)
        } -> std::same_as<std::optional<typename T::DurationType_t>>;
    {
        t.set_wait_time_between_retry(wait_time)
        } -> std::same_as<void>;
    {
        std::declval<const T &>().get_fallback_wait_time_between_retry()
        } -> std::same_as<typename T::DurationType_t>;

    //! Must have methods to get/set wait time for retry response
    //! @note This is the wait time for the downstream action to respond to the goal
    //! negative wait time means wait indefinitely, 0 means no wait
    {
        std::declval<const T &>().get_wait_time_retry_response(use_fallback_if_not_set)
        } -> std::same_as<std::optional<typename T::DurationType_t>>;
    {
        t.set_wait_time_retry_response(wait_time)
        } -> std::same_as<void>;
    {
        std::declval<const T &>().get_fallback_wait_time_retry_response()
        } -> std::same_as<typename T::DurationType_t>;
};


enum class DeliveryPrecondition {
    //! No precondition, just deliver
    NoPrecondition = 0,

    //! Any downstream must be ready
    AnyDownstreamReady = 1,

    //! All downstreams must be ready
    AllDownstreamsReady = 2,
};

enum class DeliveryResultCode {
    Success = 0,
    TriedButFailed = 1, //!< Tried to do something but failed
    NotTried = 2,       //!< Not tried to delivery because precondition is not met
};

enum class DropStrategy {
    //! Do not drop
    NoDrop = 0,

    //! Drop task/data/messages as needed
    DropAsNeeded = 1,
};


} // namespace redoxi_works

namespace JS
{
//! Type handler for time duration
template <redoxi_works::TimeDurationConcept T>
struct TypeHandler<T> {
    static Error to(T &to_type, ParseContext &context)
    {
        typename T::rep value;
        auto err = TypeHandler<typename T::rep>::to(value, context);
        if (err)
            return err;
        to_type = T(value);
        return Error::NoError;
    }
    static void from(const T &from_type, Token &token, Serializer &serializer)
    {
        typename T::rep value = from_type.count();
        TypeHandler<typename T::rep>::from(value, token, serializer);
    }
};
} // namespace JS

JS_ENUM_DECLARE_VALUE_PARSER(redoxi_works::ControlSignalCode);
JS_ENUM_DECLARE_VALUE_PARSER(redoxi_works::DeliveryPrecondition);
JS_ENUM_DECLARE_VALUE_PARSER(redoxi_works::DropStrategy);
