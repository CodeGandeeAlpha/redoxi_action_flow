#pragma once

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <chrono>
#include <optional>
#include <vector>
#include <memory>

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
    virtual std::optional<int64_t> get_number_of_enqueue_retry() const = 0;

    //! Set the maximum number of retries when enqueueing a delivery task
    virtual void set_number_of_enqueue_retry(std::optional<int64_t> number_of_retry) = 0;

    //! Get the wait time between each enqueue retry attempt
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_between_enqueue_retry() const = 0;

    //! Set the wait time between each enqueue retry attempt
    virtual void set_wait_time_between_enqueue_retry(std::optional<DefaultTimeUnit_t> wait_time) = 0;

    //! Get the maximum number of retries for delivery attempts
    virtual std::optional<int64_t> get_number_of_delivery_retry() const = 0;

    //! Set the maximum number of retries for delivery attempts
    virtual void set_number_of_delivery_retry(std::optional<int64_t> number_of_retry) = 0;

    //! Get the wait time between each delivery retry attempt
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_between_delivery_retry() const = 0;

    //! Set the wait time between each delivery retry attempt
    virtual void set_wait_time_between_delivery_retry(std::optional<DefaultTimeUnit_t> wait_time) = 0;

    //! Get the maximum wait time for a single delivery attempt
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_for_delivery_response() const = 0;

    //! Set the maximum wait time for a single delivery attempt
    virtual void set_wait_time_for_delivery_response(std::optional<DefaultTimeUnit_t> wait_time) = 0;

    //! Resolve the delivery policy to definite values, after which all the parameters are guaranteed to have values
    //! Any parameter not specified in the given policy will get either the default value or the value from the given policy (later one overrides earlier one)
    //! @param policies: the list of policies to resolve from, the one that comes later will override the one that comes earlier
    virtual void resolve_to_definite(const std::vector<const IDeliveryRetryPolicy *> &policies)
    {
        bool has_number_of_enqueue_retry = get_number_of_enqueue_retry().has_value();
        bool has_wait_time_between_enqueue_retry = get_wait_time_between_enqueue_retry().has_value();
        bool has_number_of_delivery_retry = get_number_of_delivery_retry().has_value();
        bool has_wait_time_between_delivery_retry = get_wait_time_between_delivery_retry().has_value();
        bool has_wait_time_for_delivery_response = get_wait_time_for_delivery_response().has_value();

        // Set default values if not already set
        if (!has_number_of_enqueue_retry)
            set_number_of_enqueue_retry(DefaultDeliveryRetryPolicyParams::NumberOfEnqueueRetry);
        if (!has_wait_time_between_enqueue_retry)
            set_wait_time_between_enqueue_retry(DefaultDeliveryRetryPolicyParams::WaitTimeBetweenEnqueueRetry);
        if (!has_number_of_delivery_retry)
            set_number_of_delivery_retry(DefaultDeliveryRetryPolicyParams::NumberOfDeliveryRetry);
        if (!has_wait_time_between_delivery_retry)
            set_wait_time_between_delivery_retry(DefaultDeliveryRetryPolicyParams::WaitTimeBetweenDeliveryRetry);
        if (!has_wait_time_for_delivery_response)
            set_wait_time_for_delivery_response(DefaultDeliveryRetryPolicyParams::WaitTimeForDeliveryResponse);

        // Override with values from given policies
        for (auto policy : policies) {
            // just skip if the policy is null
            if (policy == nullptr)
                continue;

            if (!has_number_of_enqueue_retry && policy->get_number_of_enqueue_retry().has_value())
                set_number_of_enqueue_retry(policy->get_number_of_enqueue_retry());
            if (!has_wait_time_between_enqueue_retry && policy->get_wait_time_between_enqueue_retry().has_value())
                set_wait_time_between_enqueue_retry(policy->get_wait_time_between_enqueue_retry());
            if (!has_number_of_delivery_retry && policy->get_number_of_delivery_retry().has_value())
                set_number_of_delivery_retry(policy->get_number_of_delivery_retry());
            if (!has_wait_time_between_delivery_retry && policy->get_wait_time_between_delivery_retry().has_value())
                set_wait_time_between_delivery_retry(policy->get_wait_time_between_delivery_retry());
            if (!has_wait_time_for_delivery_response && policy->get_wait_time_for_delivery_response().has_value())
                set_wait_time_for_delivery_response(policy->get_wait_time_for_delivery_response());
        }
    }

    //! Resolve the delivery policy with an empty list, which is equivalent to setting the policy to default values
    virtual void resolve_to_definite()
    {
        resolve_to_definite({});
    }

    //! Clone the delivery retry policy
    virtual std::shared_ptr<IDeliveryRetryPolicy> clone() const = 0;
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
    std::optional<int64_t> get_number_of_enqueue_retry() const override
    {
        return number_of_enqueue_retry;
    }

    //! Set the number of retries for enqueue attempts
    void set_number_of_enqueue_retry(std::optional<int64_t> number_of_retry) override
    {
        number_of_enqueue_retry = number_of_retry;
    }

    //! Get the wait time between each enqueue retry attempt
    std::optional<DefaultTimeUnit_t> get_wait_time_between_enqueue_retry() const override
    {
        return wait_time_between_enqueue_retry;
    }

    //! Set the wait time between each enqueue retry attempt
    void set_wait_time_between_enqueue_retry(std::optional<DefaultTimeUnit_t> wait_time) override
    {
        wait_time_between_enqueue_retry = wait_time;
    }

    //! Get the number of retries for delivery attempts
    std::optional<int64_t> get_number_of_delivery_retry() const override
    {
        return number_of_delivery_retry;
    }

    //! Set the number of retries for delivery attempts
    void set_number_of_delivery_retry(std::optional<int64_t> number_of_retry) override
    {
        number_of_delivery_retry = number_of_retry;
    }

    //! Get the wait time between each delivery retry attempt
    std::optional<DefaultTimeUnit_t> get_wait_time_between_delivery_retry() const override
    {
        return wait_time_between_delivery_retry;
    }

    //! Set the wait time between each delivery retry attempt
    void set_wait_time_between_delivery_retry(std::optional<DefaultTimeUnit_t> wait_time) override
    {
        wait_time_between_delivery_retry = wait_time;
    }

    //! Get the maximum wait time for a single delivery attempt
    std::optional<DefaultTimeUnit_t> get_wait_time_for_delivery_response() const override
    {
        return wait_time_for_delivery_response;
    }

    //! Set the maximum wait time for a single delivery attempt
    void set_wait_time_for_delivery_response(std::optional<DefaultTimeUnit_t> wait_time) override
    {
        wait_time_for_delivery_response = wait_time;
    }

    //! Clone the delivery retry policy
    std::shared_ptr<IDeliveryRetryPolicy> clone() const override
    {
        return std::make_shared<DefaultDeliveryRetryPolicy>(*this);
    }
};

} // namespace redoxi_works
