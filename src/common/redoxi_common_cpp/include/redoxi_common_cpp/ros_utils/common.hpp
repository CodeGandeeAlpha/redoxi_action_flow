#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <fmt/format.h>

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>


namespace redoxi_works
{

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
void RDX_ASSERT_CHECK_TRUE(bool condition, const std::string &format, Args &&...args)
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
void RDX_ASSERT_REQUIRE_TRUE(bool condition, const std::string &format, Args &&...args)
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
void RDX_ASSERT_TRUE(bool condition, const std::string &format, Args &&...args)
{
    rcpputils::assert_true(condition, fmt::format(format, std::forward<Args>(args)...));
}

//! Raise an error by calling RDX_ASSERT_CHECK_TRUE with a false condition
template <typename... Args>
void RDX_RAISE_ERROR(const std::string &format, Args &&...args)
{
    RDX_ASSERT_CHECK_TRUE(false, format, std::forward<Args>(args)...);
}

//! Log an info message using RCLCPP_INFO without thread ID
template <typename... Args>
void RDX_LOG_INFO(const rclcpp::Node *node, const std::string &func_name,
                  const std::string &format, Args &&...args)
{
    if constexpr (sizeof...(args) == 0) {
        RCLCPP_INFO(node->get_logger(), "[f=%s()] %s", func_name.c_str(), format.c_str());
    } else {
        RCLCPP_INFO(node->get_logger(), "[f=%s()] %s", func_name.c_str(),
                    fmt::format(format, std::forward<Args>(args)...).c_str());
    }
}

//! Log an info message using RCLCPP_INFO with or without thread ID
template <typename... Args>
void RDX_LOG_INFO(const rclcpp::Node *node, const std::string &func_name, _StrictBool with_thread_id,
                  const std::string &format, Args &&...args)
{
    if (with_thread_id) {
        if constexpr (sizeof...(args) == 0) {
            RCLCPP_INFO(node->get_logger(), "[f=%s()][tid=%lu] %s", func_name.c_str(),
                        static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                        format.c_str());
        } else {
            RCLCPP_INFO(node->get_logger(), "[f=%s()][tid=%lu] %s", func_name.c_str(),
                        static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                        fmt::format(format, std::forward<Args>(args)...).c_str());
        }
    } else {
        RDX_LOG_INFO(node, func_name, format, std::forward<Args>(args)...);
    }
}


//! Log a debug message using RCLCPP_DEBUG without thread ID
template <typename... Args>
void RDX_LOG_DEBUG(const rclcpp::Node *node, const std::string &func_name,
                   const std::string &format, Args &&...args)
{
    if constexpr (sizeof...(args) == 0) {
        RCLCPP_DEBUG(node->get_logger(), "[f=%s()] %s", func_name.c_str(), format.c_str());
    } else {
        RCLCPP_DEBUG(node->get_logger(), "[f=%s()] %s", func_name.c_str(),
                     fmt::format(format, std::forward<Args>(args)...).c_str());
    }
}

//! Log a debug message using RCLCPP_DEBUG with or without thread ID
template <typename... Args>
void RDX_LOG_DEBUG(const rclcpp::Node *node, const std::string &func_name, _StrictBool with_thread_id,
                   const std::string &format, Args &&...args)
{
    if (with_thread_id) {
        if constexpr (sizeof...(args) == 0) {
            RCLCPP_DEBUG(node->get_logger(), "[f=%s()][tid=%lu] %s", func_name.c_str(),
                         static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                         format.c_str());
        } else {
            RCLCPP_DEBUG(node->get_logger(), "[f=%s()][tid=%lu] %s", func_name.c_str(),
                         static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                         fmt::format(format, std::forward<Args>(args)...).c_str());
        }
    } else {
        RDX_LOG_DEBUG(node, func_name, format, std::forward<Args>(args)...);
    }
}

//! Log an error message using RCLCPP_ERROR without thread ID
template <typename... Args>
void RDX_LOG_ERROR(const rclcpp::Node *node, const std::string &func_name,
                   const std::string &format, Args &&...args)
{
    if constexpr (sizeof...(args) == 0) {
        RCLCPP_ERROR(node->get_logger(), "[f=%s()] %s", func_name.c_str(), format.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "[f=%s()] %s", func_name.c_str(),
                     fmt::format(format, std::forward<Args>(args)...).c_str());
    }
}

//! Log an error message using RCLCPP_ERROR with or without thread ID
template <typename... Args>
void RDX_LOG_ERROR(const rclcpp::Node *node, const std::string &func_name, _StrictBool with_thread_id,
                   const std::string &format, Args &&...args)
{
    if (with_thread_id) {
        if constexpr (sizeof...(args) == 0) {
            RCLCPP_ERROR(node->get_logger(), "[f=%s()][tid=%lu] %s", func_name.c_str(),
                         static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                         format.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "[f=%s()][tid=%lu] %s", func_name.c_str(),
                         static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                         fmt::format(format, std::forward<Args>(args)...).c_str());
        }
    } else {
        RDX_LOG_ERROR(node, func_name, format, std::forward<Args>(args)...);
    }
}

//! Log a warning message using RCLCPP_WARN without thread ID
template <typename... Args>
void RDX_LOG_WARN(const rclcpp::Node *node, const std::string &func_name,
                  const std::string &format, Args &&...args)
{
    if constexpr (sizeof...(args) == 0) {
        RCLCPP_WARN(node->get_logger(), "[f=%s()] %s", func_name.c_str(), format.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "[f=%s()] %s", func_name.c_str(),
                    fmt::format(format, std::forward<Args>(args)...).c_str());
    }
}

//! Log a warning message using RCLCPP_WARN with or without thread ID
template <typename... Args>
void RDX_LOG_WARN(const rclcpp::Node *node, const std::string &func_name, _StrictBool with_thread_id,
                  const std::string &format, Args &&...args)
{
    if (with_thread_id) {
        if constexpr (sizeof...(args) == 0) {
            RCLCPP_WARN(node->get_logger(), "[f=%s()][tid=%lu] %s", func_name.c_str(),
                        static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                        format.c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "[f=%s()][tid=%lu] %s", func_name.c_str(),
                        static_cast<uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id())),
                        fmt::format(format, std::forward<Args>(args)...).c_str());
        }
    } else {
        RDX_LOG_WARN(node, func_name, format, std::forward<Args>(args)...);
    }
}

//! Macro to get JSON parameter from node, defined as macro to avoid include nlohmann/json.hpp in this file
//! you must include nlohmann/json.hpp in the file where you use this macro
#define RDX_GET_JSON_PARAM_FROM_NODE(node)                                \
    ([](const rclcpp::Node *node) -> nlohmann::json {                     \
        rclcpp::Parameter _json_params;                                   \
        auto pkey = redoxi_works::RosParams::ParamAsJsonString::MainKey;  \
        if (!node->get_parameter(pkey, _json_params))                     \
            return nlohmann::json();                                      \
        try {                                                             \
            return nlohmann::json::parse(_json_params.as_string());       \
        } catch (const nlohmann::json::parse_error &e) {                  \
            RDX_RAISE_ERROR("Failed to parse json string: {}", e.what()); \
            return nlohmann::json();                                      \
        }                                                                 \
    })(node)


enum class ActionDownstreamResponse {
    ACCEPTED = 0,
    REJECTED = 1,
    TIMEOUT = 2,
};

//! declare some default parameters for the node, some are looked up by json string
//! return 0 if success, otherwise return error code
int declare_default_parameters_for_node(rclcpp::Node *node);

} // namespace redoxi_works
