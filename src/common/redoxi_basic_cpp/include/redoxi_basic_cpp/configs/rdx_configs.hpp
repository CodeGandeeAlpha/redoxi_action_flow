#pragma once

#include <string>
#include <chrono>
#include <numeric>

namespace redoxi_works
{
// default time unit for processing and waiting
using DefaultTimeUnit_t = std::chrono::microseconds;

/**
 * @brief Get the name of the time unit based on the std::chrono::duration type.
 *
 * @tparam DurationType The std::chrono::duration type.
 * @return constexpr const char* The name of the time unit.
 */
template <typename DurationType>
constexpr const char *_get_time_unit_name()
{
    if constexpr (std::is_same_v<DurationType, std::chrono::hours>) {
        return "hours";
    } else if constexpr (std::is_same_v<DurationType, std::chrono::minutes>) {
        return "minutes";
    } else if constexpr (std::is_same_v<DurationType, std::chrono::seconds>) {
        return "seconds";
    } else if constexpr (std::is_same_v<DurationType, std::chrono::milliseconds>) {
        return "ms(1e-3)";
    } else if constexpr (std::is_same_v<DurationType, std::chrono::microseconds>) {
        return "us(1e-6)";
    } else if constexpr (std::is_same_v<DurationType, std::chrono::nanoseconds>) {
        return "ns(1e-9)";
    } else {
        return "unknown";
    }
}

inline constexpr const char *get_default_time_unit_name()
{
    return _get_time_unit_name<DefaultTimeUnit_t>();
}

} // namespace redoxi_works