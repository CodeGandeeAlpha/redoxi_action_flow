#pragma once

#include <string>

namespace redoxi_works::logging
{
//! The format of the timestamp string, for quick use without learning the std format
enum class TimeStampFormat {
    //! year-month-day hour:minute:second.microsecond
    YYYY_MM_DD_HH_MM_SS_US,
    //! year-month-day hour:minute:second.millisecond
    YYYY_MM_DD_HH_MM_SS_MS,
    //! hour:minute:second.microsecond
    HH_MM_SS_US,
    //! hour:minute:second.millisecond
    HH_MM_SS_MS,

    //! milliseconds since the epoch
    MS_SINCE_EPOCH,
    //! seconds since the epoch
    US_SINCE_EPOCH,
};
//! Get the current timestamp in the format of YYYY-MM-DD HH:MM:SS.US
std::string get_timestamp(TimeStampFormat format = TimeStampFormat::YYYY_MM_DD_HH_MM_SS_US);
} // namespace redoxi_works::logging