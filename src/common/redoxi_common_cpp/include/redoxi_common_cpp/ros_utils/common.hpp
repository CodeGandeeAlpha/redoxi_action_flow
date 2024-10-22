#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <fmt/format.h>

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>


namespace redoxi_works
{

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

enum class ActionDownstreamResponse {
    ACCEPTED = 0,
    REJECTED = 1,
    TIMEOUT = 2,
};

//! declare some default parameters for the node, some are looked up by json string
//! return 0 if success, otherwise return error code
int declare_default_parameters_for_node(rclcpp::Node *node);

} // namespace redoxi_works
