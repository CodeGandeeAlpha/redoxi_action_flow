#include <redoxi_basic_cpp/logging/utils.hpp>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace redoxi_works::logging
{
std::string get_timestamp(TimeStampFormat format)
{
    auto current_time = std::chrono::system_clock::now();
    auto duration = current_time.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);

    auto time_t = std::chrono::system_clock::to_time_t(current_time);
    auto local_time = *std::localtime(&time_t);

    std::stringstream ss;

    switch (format) {
        case TimeStampFormat::YYYY_MM_DD_HH_MM_SS_US: {
            auto us = microseconds - seconds;
            ss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
               << '.' << std::setfill('0') << std::setw(6) << us.count();
            break;
        }
        case TimeStampFormat::YYYY_MM_DD_HH_MM_SS_MS: {
            auto ms = milliseconds - seconds;
            ss << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
               << '.' << std::setfill('0') << std::setw(3) << ms.count();
            break;
        }
        case TimeStampFormat::HH_MM_SS_US: {
            auto us = microseconds - seconds;
            ss << std::put_time(&local_time, "%H:%M:%S")
               << '.' << std::setfill('0') << std::setw(6) << us.count();
            break;
        }
        case TimeStampFormat::HH_MM_SS_MS: {
            auto ms = milliseconds - seconds;
            ss << std::put_time(&local_time, "%H:%M:%S")
               << '.' << std::setfill('0') << std::setw(3) << ms.count();
            break;
        }
        case TimeStampFormat::MS_SINCE_EPOCH:
            ss << milliseconds.count();
            break;
        case TimeStampFormat::US_SINCE_EPOCH:
            ss << microseconds.count();
            break;
    }

    return ss.str();
}
} // namespace redoxi_works::logging