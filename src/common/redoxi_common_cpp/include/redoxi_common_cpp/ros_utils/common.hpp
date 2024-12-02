#pragma once

#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <rcutils/logging_macros.h>
#include <builtin_interfaces/msg/time.hpp>
#include <fmt/format.h>

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>

//! Define log importance threshold for all severities
//! importance is an integer value, the higher the value, the more important the log message is
//! importance below the threshold will be ignored
#ifndef REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_INFO
#    define REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_INFO 0
#endif

#ifndef REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_DEBUG
#    define REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_DEBUG 0
#endif

#ifndef REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_WARN
#    define REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_WARN 0
#endif

#ifndef REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_ERROR
#    define REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_ERROR 0
#endif

#ifndef REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_FATAL
#    define REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_FATAL 0
#endif

namespace redoxi_works
{
constexpr const char *DefaultRDXLoggerName = "rdx_default_logger";

namespace DefaultParams
{

//! goal handle timeout
constexpr DefaultTimeUnit_t GoalHandleTimeout = std::chrono::milliseconds(3000);

//! publisher queue size when the topic is intended to be used for debugging
constexpr int DebugPublisherQueueSize = 10;

//! QoS for the debug publisher
const rclcpp::QoS DebugPublisherQoS = rclcpp::SensorDataQoS();

// used for performance probing
const rclcpp::QoS ProbePublisherQoS = rclcpp::RosoutQoS();
// const rclcpp::QoS DebugPublisherQoS = rclcpp::QoS(DebugPublisherQueueSize).reliable();

} // namespace DefaultParams

//! A dummy token that can be used as a placeholder for time token
struct DummyTimeToken {
};

//! A lightweight class that can only be converted to and from bool or int
class _StrictBool
{
  private:
    bool value;

  public:
    //! Default constructor
    _StrictBool()
        : value(false)
    {
    }

    //! Constructor from bool
    _StrictBool(bool b)
        : value(b)
    {
    }

    //! Constructor from int
    _StrictBool(int i)
        : value(i != 0)
    {
    }

    //! Conversion to bool
    operator bool() const
    {
        return value;
    }

    //! Conversion to int
    operator int() const
    {
        return value ? 1 : 0;
    }

    //! Assignment from bool
    _StrictBool &operator=(bool b)
    {
        value = b;
        return *this;
    }

    //! Assignment from int
    _StrictBool &operator=(int i)
    {
        value = (i != 0);
        return *this;
    }

    //! Deleted conversion operators to prevent implicit conversions to other types
    template <typename T>
    operator T() const = delete;
};


/**
 * @brief Assert that a condition is true, if not, throw an exception and terminate the program
 * @param condition the condition to assert
 * @param format the format string
 * @param args the arguments to the format string
 */
template <typename... Args>
void RDX_ASSERT_CHECK_TRUE(bool condition, fmt::format_string<Args...> format, Args &&...args)
{
    rcpputils::check_true(condition, fmt::format(format, std::forward<Args>(args)...));
}

/**
 * @brief Assert that a condition is true, if not, throw an exception
 * @param condition the condition to assert
 * @param format the format string
 * @param args the arguments to the format string
 */
template <typename... Args>
void RDX_ASSERT_REQUIRE_TRUE(bool condition, fmt::format_string<Args...> format, Args &&...args)
{
    rcpputils::require_true(condition, fmt::format(format, std::forward<Args>(args)...));
}

/**
 * @brief Assert that a condition is true, if not, log an error message
 * @param condition the condition to assert
 * @param format the format string
 * @param args the arguments to the format string
 */
template <typename... Args>
void RDX_ASSERT_TRUE(bool condition, fmt::format_string<Args...> format, Args &&...args)
{
    rcpputils::assert_true(condition, fmt::format(format, std::forward<Args>(args)...));
}

//! Raise an error by calling RDX_ASSERT_CHECK_TRUE with a false condition
template <typename... Args>
void RDX_RAISE_ERROR(fmt::format_string<Args...> format, Args &&...args)
{
    RDX_ASSERT_CHECK_TRUE(false, format, std::forward<Args>(args)...);
}

namespace log_severity
{
constexpr int DEBUG = 0;
constexpr int INFO = 1;
constexpr int WARN = 2;
constexpr int ERROR = 3;
constexpr int FATAL = 4;
} // namespace log_severity

/**
 * @brief The threshold of log importance for each severity
 * @details Only log the message if the importance is equal or greater than the threshold
 * @note Can be configured during runtime
 */
class LogImportanceThreshold
{
  public:
    static LogImportanceThreshold &getInstance()
    {
        static LogImportanceThreshold instance;
        return instance;
    }

    std::atomic<int> &operator[](size_t index)
    {
        return log_importance_threshold[index];
    }

  private:
    LogImportanceThreshold()
    {
        log_importance_threshold[0] = REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_DEBUG;
        log_importance_threshold[1] = REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_INFO;
        log_importance_threshold[2] = REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_WARN;
        log_importance_threshold[3] = REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_ERROR;
        log_importance_threshold[4] = REDOXI_WORKS_LOG_IMPORTANCE_THRESHOLD_FATAL;
    }

    std::atomic<int> log_importance_threshold[5];

    LogImportanceThreshold(const LogImportanceThreshold &) = delete;
    LogImportanceThreshold &operator=(const LogImportanceThreshold &) = delete;
};

template <typename T>
concept NodeOrLoggerConcept = requires(T t)
{
    requires std::is_same_v<std::remove_cvref_t<T>, rclcpp::Logger> ||
        std::is_convertible_v < std::remove_cvref_t<T>,
    const rclcpp::Node * > ||
        std::is_null_pointer_v<std::remove_cvref_t<T>>;
};

//! Log a message using RCLCPP macros with or without thread ID
template <NodeOrLoggerConcept NodeOrLogger, int Severity, int Importance, typename FirstArg, typename... RestArgs>
void RDX_LOG_GENERIC_(NodeOrLogger node_or_logger, const char *func_name, _StrictBool with_thread_id,
                      fmt::format_string<FirstArg, RestArgs...> format, FirstArg &&first_arg, RestArgs &&...rest_args)
{
    if (Importance < LogImportanceThreshold::getInstance()[Severity]) {
        return;
    }

    // Check if node_or_logger is nullptr
    bool is_null_logger = std::is_null_pointer_v<std::remove_cvref_t<NodeOrLogger>>;

    // Get the logger based on input type
    auto logger = [&node_or_logger]() -> rclcpp::Logger {
        if constexpr (std::is_null_pointer_v<std::remove_cvref_t<NodeOrLogger>>) {
            (void)node_or_logger;
            return rclcpp::get_logger(DefaultRDXLoggerName);
        } else if constexpr (std::is_same_v<NodeOrLogger, rclcpp::Logger>) {
            return node_or_logger;
        } else if constexpr (std::is_convertible_v<NodeOrLogger, const rclcpp::Node *>) {
            return node_or_logger->get_logger();
        } else {
            return rclcpp::get_logger(DefaultRDXLoggerName);
        }
    }();

    switch (Severity) {
        case log_severity::DEBUG:
            if (with_thread_id) {
                if (is_null_logger) {
                    RCUTILS_LOG_DEBUG("[f=%s()][tid=%lu] %s", func_name,
                                      static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                      fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_DEBUG(logger, "[f=%s()][tid=%lu] %s", func_name,
                                 static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                 fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            } else {
                if (is_null_logger) {
                    RCUTILS_LOG_DEBUG("[f=%s()] %s", func_name,
                                      fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_DEBUG(logger, "[f=%s()] %s", func_name,
                                 fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            }
            break;
        case log_severity::INFO:
            if (with_thread_id) {
                if (is_null_logger) {
                    RCUTILS_LOG_INFO("[f=%s()][tid=%lu] %s", func_name,
                                     static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                     fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_INFO(logger, "[f=%s()][tid=%lu] %s", func_name,
                                static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            } else {
                if (is_null_logger) {
                    RCUTILS_LOG_INFO("[f=%s()] %s", func_name,
                                     fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_INFO(logger, "[f=%s()] %s", func_name,
                                fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            }
            break;
        case log_severity::WARN:
            if (with_thread_id) {
                if (is_null_logger) {
                    RCUTILS_LOG_WARN("[f=%s()][tid=%lu] %s", func_name,
                                     static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                     fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_WARN(logger, "[f=%s()][tid=%lu] %s", func_name,
                                static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            } else {
                if (is_null_logger) {
                    RCUTILS_LOG_WARN("[f=%s()] %s", func_name,
                                     fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_WARN(logger, "[f=%s()] %s", func_name,
                                fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            }
            break;
        case log_severity::ERROR:
            if (with_thread_id) {
                if (is_null_logger) {
                    RCUTILS_LOG_ERROR("[f=%s()][tid=%lu] %s", func_name,
                                      static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                      fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_ERROR(logger, "[f=%s()][tid=%lu] %s", func_name,
                                 static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                 fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            } else {
                if (is_null_logger) {
                    RCUTILS_LOG_ERROR("[f=%s()] %s", func_name,
                                      fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_ERROR(logger, "[f=%s()] %s", func_name,
                                 fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            }
            break;
        case log_severity::FATAL:
            if (with_thread_id) {
                if (is_null_logger) {
                    RCUTILS_LOG_FATAL("[f=%s()][tid=%lu] %s", func_name,
                                      static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                      fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_FATAL(logger, "[f=%s()][tid=%lu] %s", func_name,
                                 static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                 fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            } else {
                if (is_null_logger) {
                    RCUTILS_LOG_FATAL("[f=%s()] %s", func_name,
                                      fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_FATAL(logger, "[f=%s()] %s", func_name,
                                 fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            }
            break;
        default:
            // Use INFO level for unknown severity
            if (with_thread_id) {
                if (is_null_logger) {
                    RCUTILS_LOG_INFO("[f=%s()][tid=%lu] %s", func_name,
                                     static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                     fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_INFO(logger, "[f=%s()][tid=%lu] %s", func_name,
                                static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                                fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            } else {
                if (is_null_logger) {
                    RCUTILS_LOG_INFO("[f=%s()] %s", func_name,
                                     fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                } else {
                    RCLCPP_INFO(logger, "[f=%s()] %s", func_name,
                                fmt::format(format, std::forward<FirstArg>(first_arg), std::forward<RestArgs>(rest_args)...).c_str());
                }
            }
            break;
    }
}

#define DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(severity, importance)                                          \
    template <int Severity = log_severity::severity, int Importance = importance,                                  \
              NodeOrLoggerConcept NodeOrLogger, typename FirstArg, typename... RestArgs>                           \
    void RDX_LOG_##severity##_##importance(NodeOrLogger logger, const char *func_name,                             \
                                           fmt::format_string<FirstArg, RestArgs...> format,                       \
                                           FirstArg &&first_arg, RestArgs &&...rest_args)                          \
    {                                                                                                              \
        RDX_LOG_GENERIC_<NodeOrLogger, Severity, Importance>(logger, func_name, false, format,                     \
                                                             std::forward<FirstArg>(first_arg),                    \
                                                             std::forward<RestArgs>(rest_args)...);                \
    }                                                                                                              \
                                                                                                                   \
    template <int Severity = log_severity::severity, int Importance = importance,                                  \
              NodeOrLoggerConcept NodeOrLogger, typename FirstArg, typename... RestArgs>                           \
    void RDX_LOG_##severity##_##importance(NodeOrLogger logger, const char *func_name, _StrictBool with_thread_id, \
                                           fmt::format_string<FirstArg, RestArgs...> format,                       \
                                           FirstArg &&first_arg, RestArgs &&...rest_args)                          \
    {                                                                                                              \
        RDX_LOG_GENERIC_<NodeOrLogger, Severity, Importance>(logger, func_name, with_thread_id, format,            \
                                                             std::forward<FirstArg>(first_arg),                    \
                                                             std::forward<RestArgs>(rest_args)...);                \
    }

#define DEFINE_RDX_LOGGING_SEVERITY(severity)                                                                \
    template <NodeOrLoggerConcept NodeOrLogger, typename FirstArg, typename... RestArgs>                     \
    void RDX_LOG_##severity(NodeOrLogger logger, const char *func_name,                                      \
                            fmt::format_string<FirstArg, RestArgs...> format,                                \
                            FirstArg &&first_arg, RestArgs &&...rest_args)                                   \
    {                                                                                                        \
        RDX_LOG_GENERIC_<NodeOrLogger, log_severity::severity, 0>(logger, func_name, false, format,          \
                                                                  std::forward<FirstArg>(first_arg),         \
                                                                  std::forward<RestArgs>(rest_args)...);     \
    }                                                                                                        \
                                                                                                             \
    template <NodeOrLoggerConcept NodeOrLogger, typename FirstArg, typename... RestArgs>                     \
    void RDX_LOG_##severity(NodeOrLogger logger, const char *func_name, _StrictBool with_thread_id,          \
                            fmt::format_string<FirstArg, RestArgs...> format,                                \
                            FirstArg &&first_arg, RestArgs &&...rest_args)                                   \
    {                                                                                                        \
        RDX_LOG_GENERIC_<NodeOrLogger, log_severity::severity, 0>(logger, func_name, with_thread_id, format, \
                                                                  std::forward<FirstArg>(first_arg),         \
                                                                  std::forward<RestArgs>(rest_args)...);     \
    }

DEFINE_RDX_LOGGING_SEVERITY(INFO)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(INFO, 0)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(INFO, 1)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(INFO, 2)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(INFO, 3)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(INFO, 4)

DEFINE_RDX_LOGGING_SEVERITY(DEBUG)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(DEBUG, 0)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(DEBUG, 1)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(DEBUG, 2)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(DEBUG, 3)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(DEBUG, 4)

DEFINE_RDX_LOGGING_SEVERITY(WARN)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(WARN, 0)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(WARN, 1)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(WARN, 2)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(WARN, 3)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(WARN, 4)

DEFINE_RDX_LOGGING_SEVERITY(ERROR)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(ERROR, 0)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(ERROR, 1)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(ERROR, 2)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(ERROR, 3)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(ERROR, 4)

DEFINE_RDX_LOGGING_SEVERITY(FATAL)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(FATAL, 0)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(FATAL, 1)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(FATAL, 2)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(FATAL, 3)
DEFINE_RDX_LOGGING_SEVERITY_WITH_IMPORTANCE(FATAL, 4)

// for verbose output
#define RDX_INFO_VERBOSE RDX_LOG_INFO_0

// for development output
#define RDX_INFO_DEV RDX_LOG_INFO_1

// for production output
#define RDX_INFO_PRODUCTION RDX_LOG_INFO_2


//! Macro to get JSON parameter from node, defined as macro to avoid include nlohmann/json.hpp in this file
//! you must include nlohmann/json.hpp in the file where you use this macro
#define RDX_GET_JSON_PARAM_FROM_NODE(node)                                      \
    ([](const rclcpp::Node *node) -> nlohmann::json {                           \
        rclcpp::Parameter _json_params;                                         \
        auto pkey = redoxi_works::RosParams::ParamAsJsonString::MainKey;        \
        if (!node->get_parameter(pkey, _json_params))                           \
            return nlohmann::json();                                            \
        try {                                                                   \
            auto str = _json_params.as_string();                                \
            return str.empty() ? nlohmann::json() : nlohmann::json::parse(str); \
        } catch (const nlohmann::json::parse_error &e) {                        \
            RDX_RAISE_ERROR("Failed to parse json string: {}", e.what());       \
            return nlohmann::json();                                            \
        }                                                                       \
    })(node)


enum class ActionDownstreamResponse {
    ACCEPTED = 0,
    REJECTED = 1,
    TIMEOUT = 2,
};

//! declare some default parameters for the node, some are looked up by json string
//! return 0 if success, otherwise return error code
int declare_default_parameters_for_node(rclcpp::Node *node);

//! Convert milliseconds to ros2 time message
builtin_interfaces::msg::Time ros2_time_msg_from_ms(double ms);

//! Convert seconds to ros2 time message
builtin_interfaces::msg::Time ros2_time_msg_from_sec(double sec);

} // namespace redoxi_works
