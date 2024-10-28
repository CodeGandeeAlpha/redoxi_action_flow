#pragma once

#include <redoxi_common_nodes/AsyncActionPortInterfaces.hpp>
#include <optional>
#include <json_struct/json_struct.h>

// default types for the async action output port
namespace redoxi_works
{

namespace AsyncActionPortTypes
{

/*!
 * @brief Interface class defining retry policy for async actions
 *
 * This interface defines methods to get and set retry parameters including:
 * - Number of retries allowed
 * - Wait time between retry attempts
 * - Wait time for retry responses
 *
 * Each parameter has three associated methods:
 * - Get the current value (returns optional)
 * - Set the current value
 * - Get the fallback value (used when current value is not set)
 */
class DefaultRetryPolicy
{
  public:
    DefaultRetryPolicy()
    {
        static_assert(RetryPolicyConcept<DefaultRetryPolicy>, "DefaultRetryPolicy must satisfy RetryPolicyConcept");
    }
    virtual ~DefaultRetryPolicy() = default;

    //! Get the current number of retry
    virtual std::optional<int64_t> get_number_of_retry() const
    {
        return m_number_of_retry;
    }

    //! Set the current number of retr
    virtual void set_number_of_retry(std::optional<int64_t> number_of_retry)
    {
        m_number_of_retry = number_of_retry;
    }

    //! Get the fallback number of retry
    virtual int64_t get_fallback_number_of_retry() const
    {
        return m_fallback_number_of_retry;
    }

    //! Get the current wait time between retries
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_between_retry() const
    {
        return m_wait_time_between_retry;
    }

    //! Set the current wait time between retries
    virtual void set_wait_time_between_retry(std::optional<DefaultTimeUnit_t> wait_time)
    {
        m_wait_time_between_retry = wait_time;
    }

    //! Get the fallback wait time between retries
    virtual DefaultTimeUnit_t get_fallback_wait_time_between_retry() const
    {
        return m_fallback_wait_time_between_retry;
    }

    //! Get the current wait time for retry response
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_retry_response() const
    {
        return m_wait_time_retry_response;
    }

    //! Set the current wait time for retry response
    virtual void set_wait_time_retry_response(std::optional<DefaultTimeUnit_t> wait_time)
    {
        m_wait_time_retry_response = wait_time;
    }

    //! Get the fallback wait time for retry response
    virtual DefaultTimeUnit_t get_fallback_wait_time_retry_response() const
    {
        return m_fallback_wait_time_retry_response;
    }

  protected:
    //! Current number of retry
    std::optional<int64_t> m_number_of_retry;
    //! Fallback number of retry
    int64_t m_fallback_number_of_retry;
    //! Current wait time between retries
    std::optional<DefaultTimeUnit_t> m_wait_time_between_retry;
    //! Fallback wait time between retries
    DefaultTimeUnit_t m_fallback_wait_time_between_retry;
    //! Current wait time for retry response
    std::optional<DefaultTimeUnit_t> m_wait_time_retry_response;
    //! Fallback wait time for retry response
    DefaultTimeUnit_t m_fallback_wait_time_retry_response;

  public:
    JS_OBJECT(JS_MEMBER_WITH_NAME(m_number_of_retry, "number_of_retry"),
              JS_MEMBER_WITH_NAME(m_fallback_number_of_retry, "fallback_number_of_retry"),
              JS_MEMBER_WITH_NAME(m_wait_time_between_retry, "wait_time_between_retry"),
              JS_MEMBER_WITH_NAME(m_fallback_wait_time_between_retry, "fallback_wait_time_between_retry"),
              JS_MEMBER_WITH_NAME(m_wait_time_retry_response, "wait_time_retry_response"),
              JS_MEMBER_WITH_NAME(m_fallback_wait_time_retry_response, "fallback_wait_time_retry_response"));
};

} // namespace AsyncActionPortTypes

} // namespace redoxi_works
