#ifndef REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_
#define REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_

#include "redoxi_common_cpp/visibility_control.h"

namespace redoxi_works
{
// default timeout in ms
const double DefaultTimeoutMs = 10000;
// default max number of retries
const int DefaultMaxNumberOfRetries = 10;
// how many ms to wait before next _step()
const double DefaultNodeStepIntervalMs = 1;
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
     * @return the maximum number of retries, -1 means no limit
     */
    virtual int get_max_number_of_retries() = 0;

    /**
     * @brief set the maximum number of retries
     *
     * @param max_retries the maximum number of retries, -1 means no limit
     */
    virtual void set_max_number_of_retries(int max_retries) = 0;

    /**
     * @brief get the maximum wait time in ms
     *
     * @return the maximum wait time in ms, -1 means no limit
     */
    virtual int get_max_wait_time_ms() = 0;

    /**
     * @brief set the maximum wait time in ms
     *
     * @param max_wait_time_ms the maximum wait time in ms, -1 means no limit
     */
    virtual void set_max_wait_time_ms(int max_wait_time_ms) = 0;
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
    int max_wait_time_ms = DefaultTimeoutMs;

  public:
    int get_max_number_of_retries() override
    {
        return max_number_of_retries;
    }

    void set_max_number_of_retries(int max_retries) override
    {
        max_number_of_retries = max_retries;
    }

    int get_max_wait_time_ms() override
    {
        return max_wait_time_ms;
    }

    void set_max_wait_time_ms(int max_wait_time_ms) override
    {
        this->max_wait_time_ms = max_wait_time_ms;
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
const int INITIALIZED = 1;
const int OPENED = 2;
const int STARTED = 3;
const int STOPPED = 4;
const int CLOSED = 5;

// reserved status code, your custom status code should be greater than this
const int MAX_RESERVED_STATUS = 10000;
}; // namespace NodeStatusCode

namespace SignalCode
{
const int RUN = 0;
const int FLUSH = 1;
const int TERMINATE = 2;
}; // namespace SignalCode


} // namespace redoxi_works

#endif // REDOXI_COMMON_CPP__REDOXI_COMMON_CPP_HPP_
