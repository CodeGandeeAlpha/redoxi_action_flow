#pragma once

#include <string>
#include <optional>
#include <concepts>
#include <utility>
#include <type_traits>
#include <chrono>
#include <limits>
#include <map>

#include <unique_identifier_msgs/msg/uuid.hpp>
#include <boost/uuid/uuid.hpp>
#include <json_struct/json_struct.h>

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>


namespace redoxi_works
{

//! Control signal codes used to indicate special actions in messages
//! @note If a message contains other messages that contain control signals,
//!       only the top level control message is used
enum class ControlSignalCode {
    Normal = 0,    //!< Normal signal, no special action needed
    Ping = 1,      //!< Ping signal - downstream should reply but not process data
    Flush = 2,     //!< Flush signal - finish processing previous data and process new data in clean state
    Reset = 3,     //!< Reset signal - reset to initial state as if previous data never existed
    Terminate = 4, //!< Terminate signal - no more data will be sent after this

    // leave for custom use
    Custom_1 = 10001,
    Custom_2 = 10002,
    Custom_3 = 10003,
    Custom_4 = 10004,
    Custom_5 = 10005,
    Custom_6 = 10006,
    Custom_7 = 10007,
    Custom_8 = 10008,
    Custom_9 = 10009,

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
template <RosActionConcept ActionType>
struct NoneActionDataTrait {
    using ActionType_t = ActionType;
    using Goal_t = typename ActionType_t::Goal;
    using Result_t = typename ActionType_t::Result;
    using Feedback_t = typename ActionType_t::Feedback;

    static ControlSignalCode get_control_signal_code(const Goal_t &)
    {
        return ControlSignalCode::Unknown;
    }

    static void mark_with_control_signal(Goal_t &, ControlSignalCode)
    {
    }

    static boost::uuids::uuid get_uuid(const Goal_t &)
    {
        return boost::uuids::uuid();
    }

    static void set_uuid(Goal_t &, const boost::uuids::uuid &)
    {
    }
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
    //! Not care about precondition, let the system decide
    DontCare = 0,

    //! No precondition, just deliver
    NoPrecondition = 1,

    //! Any downstream must be ready
    AnyDownstreamReady = 2,

    //! All downstreams must be ready
    AllDownstreamsReady = 3,

    //! leave for custom use
    Custom_1 = 10001,
    Custom_2 = 10002,
    Custom_3 = 10003,
    Custom_4 = 10004,
    Custom_5 = 10005,
    Custom_6 = 10006,
    Custom_7 = 10007,
    Custom_8 = 10008,
    Custom_9 = 10009,
};

enum class DeliveryResultCode {
    Success = 0,
    TriedButFailed = 1, //!< Tried to do something but failed
    NotTried = 2,       //!< Not tried to delivery because precondition is not met
};

enum class DropStrategy {
    //! Not care about drop strategy, let the system decide
    DontCare = 0,

    //! Do not drop
    NoDrop = 1,

    //! Drop task/data/messages as needed
    DropAsNeeded = 2,

    //! leave for custom use
    Custom_1 = 10001,
    Custom_2 = 10002,
    Custom_3 = 10003,
    Custom_4 = 10004,
    Custom_5 = 10005,
    Custom_6 = 10006,
    Custom_7 = 10007,
    Custom_8 = 10008,
    Custom_9 = 10009,
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
        if (err != Error::NoError)
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

// JS_ENUM_DECLARE_VALUE_PARSER(redoxi_works::ControlSignalCode);
// JS_ENUM_DECLARE_VALUE_PARSER(redoxi_works::DeliveryPrecondition);
// JS_ENUM_DECLARE_VALUE_PARSER(redoxi_works::DropStrategy);

namespace JS
{

//! Concept to check if a type is an enum string mapper
template <typename T>
concept EnumStringMapperConcept = requires(T e)
{
    typename T::EnumType_t;
    requires std::is_enum_v<typename T::EnumType_t>;

    //! Get mapping from string to enum value
    {
        T::get_str_to_enum()
        } -> std::same_as<const std::map<std::string, typename T::EnumType_t> &>;

    //! Get mapping from enum value to string
    {
        T::get_enum_to_str()
        } -> std::same_as<const std::map<typename T::EnumType_t, std::string> &>;
};

template <EnumStringMapperConcept Mapper>
struct TypeHandlerWithMapper {
    using EnumType_t = typename Mapper::EnumType_t;
    using Mapper_t = Mapper;

    static inline Error to(EnumType_t &to_type, ParseContext &context)
    {
        //! First try to parse as string
        const auto &str_to_enum = Mapper::get_str_to_enum();
        std::string str;
        Error err = TypeHandler<std::string>::to(str, context);

        if (err == Error::NoError) {
            //! Convert to lower case for case-insensitive comparison
            std::transform(str.begin(), str.end(), str.begin(), ::tolower);

            auto it = str_to_enum.find(str);
            if (it != str_to_enum.end()) {
                to_type = it->second;
                return Error::NoError;
            }
            return Error::IllegalDataValue;
        }
        return err;
    }

    static inline void from(const EnumType_t &from_type, Token &token, Serializer &serializer)
    {
        const auto &enum_to_str = Mapper::get_enum_to_str();

        //! Convert to string using enum_to_str map
        auto it = enum_to_str.find(from_type);
        if (it != enum_to_str.end()) {
            TypeHandler<std::string>::from(it->second, token, serializer);
        }
    }
};

struct ControlSignalCodeMapper {
    using EnumType_t = redoxi_works::ControlSignalCode;
    using Mapper_t = ControlSignalCodeMapper;
    ControlSignalCodeMapper()
    {
        static_assert(EnumStringMapperConcept<ControlSignalCodeMapper>, "ControlSignalCodeMapper must satisfy EnumStringMapperConcept");
    }

    static inline const std::map<std::string, EnumType_t> str_to_enum = {
        {"normal", EnumType_t::Normal},
        {"ping", EnumType_t::Ping},
        {"flush", EnumType_t::Flush},
        {"reset", EnumType_t::Reset},
        {"terminate", EnumType_t::Terminate},
    };

    static inline const std::map<EnumType_t, std::string> enum_to_str = {
        {EnumType_t::Normal, "normal"},
        {EnumType_t::Ping, "ping"},
        {EnumType_t::Flush, "flush"},
        {EnumType_t::Reset, "reset"},
        {EnumType_t::Terminate, "terminate"},
    };

    static inline const std::map<std::string, EnumType_t> &get_str_to_enum()
    {
        return str_to_enum;
    }

    static inline const std::map<EnumType_t, std::string> &get_enum_to_str()
    {
        return enum_to_str;
    }
};


template <>
struct TypeHandler<redoxi_works::ControlSignalCode> : public TypeHandlerWithMapper<ControlSignalCodeMapper> {
};

struct DeliveryPreconditionMapper {
    using EnumType_t = redoxi_works::DeliveryPrecondition;
    using Mapper_t = DeliveryPreconditionMapper;
    DeliveryPreconditionMapper()
    {
        static_assert(EnumStringMapperConcept<DeliveryPreconditionMapper>, "DeliveryPreconditionMapper must satisfy EnumStringMapperConcept");
    }

    static inline const std::map<std::string, EnumType_t> str_to_enum = {
        {"dont_care", EnumType_t::DontCare},
        {"no_precondition", EnumType_t::NoPrecondition},
        {"any_downstream_ready", EnumType_t::AnyDownstreamReady},
        {"all_downstreams_ready", EnumType_t::AllDownstreamsReady},
        {"custom_1", EnumType_t::Custom_1},
        {"custom_2", EnumType_t::Custom_2},
        {"custom_3", EnumType_t::Custom_3},
        {"custom_4", EnumType_t::Custom_4},
        {"custom_5", EnumType_t::Custom_5},
        {"custom_6", EnumType_t::Custom_6},
        {"custom_7", EnumType_t::Custom_7},
        {"custom_8", EnumType_t::Custom_8},
        {"custom_9", EnumType_t::Custom_9}};

    static inline const std::map<EnumType_t, std::string> enum_to_str = {
        {EnumType_t::DontCare, "dont_care"},
        {EnumType_t::NoPrecondition, "no_precondition"},
        {EnumType_t::AnyDownstreamReady, "any_downstream_ready"},
        {EnumType_t::AllDownstreamsReady, "all_downstreams_ready"},
        {EnumType_t::Custom_1, "custom_1"},
        {EnumType_t::Custom_2, "custom_2"},
        {EnumType_t::Custom_3, "custom_3"},
        {EnumType_t::Custom_4, "custom_4"},
        {EnumType_t::Custom_5, "custom_5"},
        {EnumType_t::Custom_6, "custom_6"},
        {EnumType_t::Custom_7, "custom_7"},
        {EnumType_t::Custom_8, "custom_8"},
        {EnumType_t::Custom_9, "custom_9"},
    };

    static inline const std::map<std::string, EnumType_t> &get_str_to_enum()
    {
        return str_to_enum;
    }

    static inline const std::map<EnumType_t, std::string> &get_enum_to_str()
    {
        return enum_to_str;
    }
};


template <>
struct TypeHandler<redoxi_works::DeliveryPrecondition> : public TypeHandlerWithMapper<DeliveryPreconditionMapper> {
};

//! String mapper for DropStrategy enum
struct DropStrategyMapper {
    using EnumType_t = redoxi_works::DropStrategy;

    static inline void validate()
    {
        static_assert(EnumStringMapperConcept<DropStrategyMapper>, "DropStrategyMapper must satisfy EnumStringMapperConcept");
    }

    static inline const std::map<std::string, EnumType_t> str_to_enum = {
        {"dont_care", EnumType_t::DontCare},
        {"no_drop", EnumType_t::NoDrop},
        {"drop_as_needed", EnumType_t::DropAsNeeded},
        {"custom_1", EnumType_t::Custom_1},
        {"custom_2", EnumType_t::Custom_2},
        {"custom_3", EnumType_t::Custom_3},
        {"custom_4", EnumType_t::Custom_4},
        {"custom_5", EnumType_t::Custom_5},
        {"custom_6", EnumType_t::Custom_6},
        {"custom_7", EnumType_t::Custom_7},
        {"custom_8", EnumType_t::Custom_8},
        {"custom_9", EnumType_t::Custom_9}};

    static inline const std::map<EnumType_t, std::string> enum_to_str = {
        {EnumType_t::DontCare, "dont_care"},
        {EnumType_t::NoDrop, "no_drop"},
        {EnumType_t::DropAsNeeded, "drop_as_needed"},
        {EnumType_t::Custom_1, "custom_1"},
        {EnumType_t::Custom_2, "custom_2"},
        {EnumType_t::Custom_3, "custom_3"},
        {EnumType_t::Custom_4, "custom_4"},
        {EnumType_t::Custom_5, "custom_5"},
        {EnumType_t::Custom_6, "custom_6"},
        {EnumType_t::Custom_7, "custom_7"},
        {EnumType_t::Custom_8, "custom_8"},
        {EnumType_t::Custom_9, "custom_9"},
    };

    static inline const std::map<std::string, EnumType_t> &get_str_to_enum()
    {
        return str_to_enum;
    }

    static inline const std::map<EnumType_t, std::string> &get_enum_to_str()
    {
        return enum_to_str;
    }
};

template <>
struct TypeHandler<redoxi_works::DropStrategy> : public TypeHandlerWithMapper<DropStrategyMapper> {
};


} // namespace JS
