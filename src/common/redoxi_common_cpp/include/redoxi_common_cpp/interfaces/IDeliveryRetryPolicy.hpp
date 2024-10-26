#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <chrono>
#include <optional>

namespace redoxi_works
{

namespace DefaultDeliveryRetryPolicyParams
{
constexpr DefaultTimeUnit_t WaitTimeBetweenEnqueueRetry = std::chrono::milliseconds(5);
constexpr DefaultTimeUnit_t WaitTimeBetweenDeliveryRetry = std::chrono::milliseconds(10);
constexpr DefaultTimeUnit_t WaitTimeForDeliveryResponse = std::chrono::milliseconds(50);

constexpr int64_t NumberOfEnqueueRetry = 10;
constexpr int64_t NumberOfDeliveryRetry = 10;
} // namespace DefaultDeliveryRetryPolicyParams

//! Interface for retry policy during data delivery
class IDeliveryRetryPolicy
{
  public:
    virtual ~IDeliveryRetryPolicy() = default;

    //! Get the maximum number of retries when enqueueing a delivery task
    virtual std::optional<int64_t> get_number_of_enqueue_retry() = 0;

    //! Set the maximum number of retries when enqueueing a delivery task
    virtual void set_number_of_enqueue_retry(std::optional<int64_t> number_of_retry) = 0;

    //! Get the wait time between each enqueue retry attempt
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_between_enqueue_retry() = 0;

    //! Set the wait time between each enqueue retry attempt
    virtual void set_wait_time_between_enqueue_retry(std::optional<DefaultTimeUnit_t> wait_time) = 0;

    //! Get the maximum number of retries for delivery attempts
    virtual std::optional<int64_t> get_number_of_delivery_retry() = 0;

    //! Set the maximum number of retries for delivery attempts
    virtual void set_number_of_delivery_retry(std::optional<int64_t> number_of_retry) = 0;

    //! Get the wait time between each delivery retry attempt
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_between_delivery_retry() = 0;

    //! Set the wait time between each delivery retry attempt
    virtual void set_wait_time_between_delivery_retry(std::optional<DefaultTimeUnit_t> wait_time) = 0;

    //! Get the maximum wait time for a single delivery attempt
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_for_delivery_response() = 0;

    //! Set the maximum wait time for a single delivery attempt
    virtual void set_wait_time_for_delivery_response(std::optional<DefaultTimeUnit_t> wait_time) = 0;
};

//! Default retry policy
class DefaultDeliveryRetryPolicy : public IDeliveryRetryPolicy
{
  public:
    std::optional<int64_t> number_of_enqueue_retry;
    std::optional<DefaultTimeUnit_t> wait_time_between_enqueue_retry;
    std::optional<int64_t> number_of_delivery_retry;
    std::optional<DefaultTimeUnit_t> wait_time_between_delivery_retry;
    std::optional<DefaultTimeUnit_t> wait_time_for_delivery_response;

    //! Get the number of retries for enqueue attempts
    std::optional<int64_t> get_number_of_enqueue_retry() override
    {
        return number_of_enqueue_retry;
    }

    //! Set the number of retries for enqueue attempts
    void set_number_of_enqueue_retry(std::optional<int64_t> number_of_retry) override
    {
        number_of_enqueue_retry = number_of_retry;
    }

    //! Get the wait time between each enqueue retry attempt
    std::optional<DefaultTimeUnit_t> get_wait_time_between_enqueue_retry() override
    {
        return wait_time_between_enqueue_retry;
    }

    //! Set the wait time between each enqueue retry attempt
    void set_wait_time_between_enqueue_retry(std::optional<DefaultTimeUnit_t> wait_time) override
    {
        wait_time_between_enqueue_retry = wait_time;
    }

    //! Get the number of retries for delivery attempts
    std::optional<int64_t> get_number_of_delivery_retry() override
    {
        return number_of_delivery_retry;
    }

    //! Set the number of retries for delivery attempts
    void set_number_of_delivery_retry(std::optional<int64_t> number_of_retry) override
    {
        number_of_delivery_retry = number_of_retry;
    }

    //! Get the wait time between each delivery retry attempt
    std::optional<DefaultTimeUnit_t> get_wait_time_between_delivery_retry() override
    {
        return wait_time_between_delivery_retry;
    }

    //! Set the wait time between each delivery retry attempt
    void set_wait_time_between_delivery_retry(std::optional<DefaultTimeUnit_t> wait_time) override
    {
        wait_time_between_delivery_retry = wait_time;
    }

    //! Get the maximum wait time for a single delivery attempt
    std::optional<DefaultTimeUnit_t> get_wait_time_for_delivery_response() override
    {
        return wait_time_for_delivery_response;
    }

    //! Set the maximum wait time for a single delivery attempt
    void set_wait_time_for_delivery_response(std::optional<DefaultTimeUnit_t> wait_time) override
    {
        wait_time_for_delivery_response = wait_time;
    }
};

} // namespace redoxi_works
