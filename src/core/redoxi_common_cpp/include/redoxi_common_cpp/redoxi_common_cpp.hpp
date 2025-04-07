#ifndef REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_
#define REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_

#include "redoxi_common_cpp/visibility_control.h"
#include <string>
#include <numeric>

#include <redoxi_basic_cpp/configs/rdx_configs.hpp>
#include <redoxi_basic_cpp/types/rdx_uuid.hpp>
#include <redoxi_basic_cpp/concepts/basic_concepts.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace redoxi_works
{
// default timeout in ms
const double DefaultTimeoutMs = 10000;
// default max number of retries
const int DefaultMaxNumberOfRetries = 10;
// how many ms to wait before next _step()
const double DefaultNodeStepIntervalMs = 1;

// default image encodings, in ros2 sensor_msgs::image_encodings format
constexpr const std::string_view DefaultColorImageEncoding = "rgb8";
constexpr const std::string_view DefaultMonoImageEncoding = "mono8";

// globally accessible parameters in ROS related to this application
namespace RosParams
{

namespace Keys
{
const std::string v6d_socket_name = "v6d_socket_name";

} // namespace Keys

namespace ParamAsJsonString
{

/**
 * @brief In each node, you can look for a parameter named "param_as_json_string"
 *
 * The value of this parameter should be a JSON string, and it will be parsed into a JSON object,
 * which contains the parameters for the node.
 */
const std::string MainKey = "param_as_json_string";

/**
 * @brief json[DeclareParams]={"param_name_1": "param_value_1", "param_name_2": "param_value_2", ...}, which
 *        defines the parameters to be declared in the ros node
 */
const std::string DeclareParams = "declare_params";

} // namespace ParamAsJsonString

} // namespace RosParams

class IOpenCloseProtocol
{
  public:
    virtual ~IOpenCloseProtocol() = default;

    // return 0 if success, otherwise return error code
    virtual int open() = 0;
    virtual int start() = 0;
    virtual int stop() = 0;
    virtual int close() = 0;
};

class IStartStopProtocol
{
  public:
    virtual ~IStartStopProtocol() = default;

    // return 0 if success, otherwise return error code
    virtual int start() = 0;
    virtual int stop() = 0;
};

namespace ReturnCode
{
const int SUCCESS = 0;
const int REJECTED = 1;
const int ERROR = -1;

// reserved status code, your custom status code should be greater than this
const int MAX_RESERVED_STATUS = 10000;
}; // namespace ReturnCode


// // for ROS lifecycle node, init()->CLOSED->open()->STOPPED->start()->STARTED->stop()->STOPPED->close()->CLOSED
// // for ROS node, BEFORE_INIT->init()->CLOSED->open()->OPENED->start()->STARTED->stop()->STOPPED->close()->CLOSED
// namespace NodeStatusCode
// {
// const int BEFORE_INIT = 0;
// const int OPENED = 2;     // open() is called
// const int STARTED = 3;    // step() is running
// const int STOPPED = 4;    // you can still start it again
// const int CLOSED = 5;     // you can still open it again
// const int TERMINATED = 6; // if you end up here, you can no longer change it to anything else

// const int INITIALIZED = 1; // NOT USED anymore

// // reserved status code, your custom status code should be greater than this
// const int MAX_RESERVED_STATUS = 10000;
// }; // namespace NodeStatusCode

// for ROS lifecycle node, init()->CLOSED->open()->STOPPED->start()->STARTED->stop()->STOPPED->close()->CLOSED
// for ROS node, BEFORE_INIT->init()->CLOSED->open()->OPENED->start()->STARTED->stop()->STOPPED->close()->CLOSED
namespace NodeStatusCode
{
const int BEFORE_INIT = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
// const int OPENED = 2;                                                       // open() is called
const int STARTED = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;       // step() is running
const int STOPPED = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;     // you can still start it again
const int CLOSED = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;  // you can still open it again
const int TERMINATED = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED; // if you end up here, you can no longer change it to anything else

// const int INITIALIZED = 1; // NOT USED anymore

// reserved status code, your custom status code should be greater than this
const int MAX_RESERVED_STATUS = 10000;
}; // namespace NodeStatusCode

inline constexpr const char *NodeStatusCodeToString(int status_code)
{
    switch (status_code) {
        case NodeStatusCode::BEFORE_INIT:
            return "BEFORE_INIT";
        case NodeStatusCode::STARTED:
            return "STARTED";
        case NodeStatusCode::STOPPED:
            return "STOPPED";
        case NodeStatusCode::CLOSED:
            return "CLOSED";
        default:
            return "UNKNOWN_STATUS";
    }
}

namespace SignalCode
{
const int RUN = 0;
const int FLUSH = 1;
const int TERMINATE = 2;
}; // namespace SignalCode


} // namespace redoxi_works

#endif // REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_
