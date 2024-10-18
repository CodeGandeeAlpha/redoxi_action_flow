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

enum class ActionDownstreamResponse {
    ACCEPTED = 0,
    REJECTED = 1,
    TIMEOUT = 2,
};

} // namespace redoxi_works
