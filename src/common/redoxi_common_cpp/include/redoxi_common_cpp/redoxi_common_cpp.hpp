#ifndef REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_
#define REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_

#include "redoxi_common_cpp/visibility_control.h"
#include <string>
#include <chrono>
#include <type_traits>

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
const DefaultTimeUnit_t MaxTimeout = std::chrono::seconds(30);

//! maximum number of retries for any operation that may fail
const int MaxNumberOfRetries = 100;

//! interval between each retry for any operation that may fail
const DefaultTimeUnit_t FastRetryInterval = std::chrono::microseconds(100);
const DefaultTimeUnit_t SlowRetryInterval = std::chrono::milliseconds(1);

//! wait time for ping action
const DefaultTimeUnit_t PingActionWaitTime = std::chrono::milliseconds(10);

} // namespace DefaultParams

// globally accessible parameters in ROS related to this application
namespace RosParams
{

namespace Keys
{
const std::string v6d_socket_name = "v6d_socket_name";
} // namespace Keys

} // namespace RosParams

class IOpenCloseProtocol
{
  public:
    virtual ~IOpenCloseProtocol()
    {
    }

    // return 0 if success, otherwise return error code
    virtual int open() = 0;
    virtual int start() = 0;
    virtual int stop() = 0;
    virtual int close() = 0;
};

class IStartStopProtocol
{
  public:
    virtual ~IStartStopProtocol()
    {
    }

    // return 0 if success, otherwise return error code
    virtual int start() = 0;
    virtual int stop() = 0;
};

/**
 * @brief Interface for retry strategy
 *
 * @details This interface is used to define the retry strategy for a task.
 *
 * @note This interface is designed to be as simple as a struct, so the members are mostly public.
 */
class IRetryStrategy
{
  public:
    virtual ~IRetryStrategy() = default;

    /**
     * @brief get the maximum number of retries
     *
     * @return the maximum number of retries, negative means no limit
     */
    virtual int get_max_number_of_retries() = 0;

    /**
     * @brief set the maximum number of retries
     *
     * @param max_retries the maximum number of retries, negative means no limit
     */
    virtual void set_max_number_of_retries(int max_retries) = 0;

    /**
     * @brief get the maximum wait time for each retry
     *
     * @return the maximum wait time for each retry, 0 means no waiting, negative means wait indefinitely
     */
    virtual DefaultTimeUnit_t get_wait_time_for_retry() = 0;

    /**
     * @brief set the maximum wait time for each retry
     *
     * @param wait_time_for_retry the maximum wait time for each retry, 0 means no waiting, negative means wait indefinitely
     */
    virtual void set_wait_time_for_retry(DefaultTimeUnit_t wait_time_for_retry) = 0;
};

/**
 * @brief Default retry strategy
 *
 * @details This is the default retry strategy, which is a simple struct.
 */
class DefaultRetryStrategy : public IRetryStrategy
{
    // this class is ment to be as simple as a struct, so the members are public
  public:
    int max_number_of_retries = DefaultMaxNumberOfRetries;
    DefaultTimeUnit_t max_wait_time_ms = std::chrono::milliseconds((int64_t)DefaultTimeoutMs);

  public:
    int get_max_number_of_retries() override
    {
        return max_number_of_retries;
    }

    void set_max_number_of_retries(int max_retries) override
    {
        max_number_of_retries = max_retries;
    }

    DefaultTimeUnit_t get_wait_time_for_retry() override
    {
        return max_wait_time_ms;
    }

    void set_wait_time_for_retry(DefaultTimeUnit_t wait_time_for_retry) override
    {
        this->max_wait_time_ms = wait_time_for_retry;
    }
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
