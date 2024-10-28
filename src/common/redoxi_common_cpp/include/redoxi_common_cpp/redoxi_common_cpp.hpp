#ifndef REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_
#define REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_

#include "redoxi_common_cpp/visibility_control.h"
#include <string>
#include <chrono>
#include <type_traits>
#include <numeric>

namespace redoxi_works
{
// default timeout in ms
const double DefaultTimeoutMs = 10000;
// default max number of retries
const int DefaultMaxNumberOfRetries = 10;
// how many ms to wait before next _step()
const double DefaultNodeStepIntervalMs = 1;

// default time unit for processing and waiting
using DefaultTimeUnit_t = std::chrono::microseconds;

namespace DefaultParams
{

//! maximum timeout for any operation that may block
constexpr DefaultTimeUnit_t MaxTimeout = std::chrono::seconds(30);

//! interval between each retry for any operation that may fail
constexpr DefaultTimeUnit_t PingActionRetryInterval = std::chrono::milliseconds(5);

//! goal handle timeout
constexpr DefaultTimeUnit_t GoalHandleTimeout = std::chrono::milliseconds(2000);

} // namespace DefaultParams

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

namespace NodeStatusCode
{
const int BEFORE_INIT = 0;
const int OPENED = 2;
const int STARTED = 3;
const int STOPPED = 4;
const int CLOSED = 5;

const int INITIALIZED = 1; // NOT USED anymore

// reserved status code, your custom status code should be greater than this
const int MAX_RESERVED_STATUS = 10000;
}; // namespace NodeStatusCode

namespace SignalCode
{
const int RUN = 0;
const int FLUSH = 1;
const int TERMINATE = 2;
}; // namespace SignalCode

// hacky stuff
namespace hacky
{

/**
 * @brief Type trait to check if a type is a std::chrono::duration
 * @details Usage:
 * template <typename T, typename = std::enable_if_t<is_duration<T>::value>>
 * void function(T duration) { ... }
 *
 * Or in C++20:
 * template <typename T>
 *     requires is_duration<T>::value
 * void function(T duration) { ... }
 */
template <typename T, typename = void>
struct is_duration : std::false_type {
};

/**
 * @brief Specialization for types that satisfy the requirements of std::chrono::duration
 * @tparam T The type to check
 * @details This trait is true for types that have a count() method, a rep type,
 *          a period type, and are convertible to std::chrono::duration
 */
template <typename T>
struct is_duration<T, std::void_t<decltype(std::declval<T>().count()),
                                  typename T::rep,
                                  typename T::period>> : std::is_convertible<T, std::chrono::duration<typename T::rep,
                                                                                                      typename T::period>> {
};

} // namespace hacky


} // namespace redoxi_works

#endif // REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_
