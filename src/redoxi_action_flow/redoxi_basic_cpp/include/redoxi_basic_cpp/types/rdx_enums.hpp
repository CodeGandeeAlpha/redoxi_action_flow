#pragma once

#include <limits>
#include <cstdint>

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

//! Convert control signal code to string
inline constexpr const char *control_signal_code_to_string(ControlSignalCode code)
{
    switch (code) {
        case ControlSignalCode::Normal:
            return "Normal";
        case ControlSignalCode::Ping:
            return "Ping";
        case ControlSignalCode::Flush:
            return "Flush";
        case ControlSignalCode::Reset:
            return "Reset";
        case ControlSignalCode::Terminate:
            return "Terminate";
        case ControlSignalCode::Custom_1:
            return "Custom_1";
        case ControlSignalCode::Custom_2:
            return "Custom_2";
        case ControlSignalCode::Custom_3:
            return "Custom_3";
        case ControlSignalCode::Custom_4:
            return "Custom_4";
        case ControlSignalCode::Custom_5:
            return "Custom_5";
        case ControlSignalCode::Custom_6:
            return "Custom_6";
        case ControlSignalCode::Custom_7:
            return "Custom_7";
        case ControlSignalCode::Custom_8:
            return "Custom_8";
        case ControlSignalCode::Custom_9:
            return "Custom_9";
        case ControlSignalCode::Unknown:
            return "Unknown";
        default:
            return "Invalid";
    }
}

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