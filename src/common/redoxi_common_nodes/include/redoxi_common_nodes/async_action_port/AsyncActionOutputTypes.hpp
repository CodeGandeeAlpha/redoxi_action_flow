#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionPortInterfaces.hpp>
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
template <TimeDurationConcept TimeUnitType>
class DefaultRetryPolicy
{
  public:
    using DurationType_t = TimeUnitType;

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
    virtual std::optional<DurationType_t> get_wait_time_between_retry() const
    {
        return m_wait_time_between_retry;
    }

    //! Set the current wait time between retries
    virtual void set_wait_time_between_retry(std::optional<DurationType_t> wait_time)
    {
        m_wait_time_between_retry = wait_time;
    }

    //! Get the fallback wait time between retries
    virtual DurationType_t get_fallback_wait_time_between_retry() const
    {
        return m_fallback_wait_time_between_retry;
    }

    //! Get the current wait time for retry response
    virtual std::optional<DurationType_t> get_wait_time_retry_response() const
    {
        return m_wait_time_retry_response;
    }

    //! Set the current wait time for retry response
    virtual void set_wait_time_retry_response(std::optional<DurationType_t> wait_time)
    {
        m_wait_time_retry_response = wait_time;
    }

    //! Get the fallback wait time for retry response
    virtual DurationType_t get_fallback_wait_time_retry_response() const
    {
        return m_fallback_wait_time_retry_response;
    }

  protected:
    //! Current number of retry
    std::optional<int64_t> m_number_of_retry;
    //! Fallback number of retry
    int64_t m_fallback_number_of_retry;
    //! Current wait time between retries
    std::optional<DurationType_t> m_wait_time_between_retry;
    //! Fallback wait time between retries
    DurationType_t m_fallback_wait_time_between_retry;
    //! Current wait time for retry response
    std::optional<DurationType_t> m_wait_time_retry_response;
    //! Fallback wait time for retry response
    DurationType_t m_fallback_wait_time_retry_response;

  public:
    JS_OBJECT(JS_MEMBER_WITH_NAME(m_number_of_retry, "number_of_retry"),
              JS_MEMBER_WITH_NAME(m_fallback_number_of_retry, "fallback_number_of_retry"),
              JS_MEMBER_WITH_NAME(m_wait_time_between_retry, "wait_time_between_retry"),
              JS_MEMBER_WITH_NAME(m_fallback_wait_time_between_retry, "fallback_wait_time_between_retry"),
              JS_MEMBER_WITH_NAME(m_wait_time_retry_response, "wait_time_retry_response"),
              JS_MEMBER_WITH_NAME(m_fallback_wait_time_retry_response, "fallback_wait_time_retry_response"));
};


template <RosActionConcept T>
class DefaultTargetData
{
  public:
    //! The ROS message type that this target data wraps
    using ActionType_t = T;
    using Goal_t = typename ActionType_t::Goal;

    DefaultTargetData()
    {
        static_assert(DeliveryTargetDataConcept<DefaultTargetData>, "DefaultTargetData must satisfy DeliveryTargetDataConcept");
    }
    virtual ~DefaultTargetData() = default;
    DefaultTargetData(const DefaultTargetData &) = default;

    virtual DefaultTargetData &operator=(const DefaultTargetData &) = default;

    //! Get the underlying ROS message
    virtual const Goal_t *get_goal() const
    {
        return m_goal.has_value() ? &m_goal.value() : nullptr;
    }

    //! Set the ROS message
    virtual void set_goal(const Goal_t &goal)
    {
        m_goal = goal;
    }

    //! Check if this is a ping signal
    virtual bool is_ping() const
    {
        return m_is_ping;
    }

    //! Set ping flag
    virtual void set_ping(bool is_ping)
    {
        m_is_ping = is_ping;
    }

  protected:
    std::optional<Goal_t> m_goal;
    bool m_is_ping{false};
};

struct DefaultStampData {
};

template <DeliverySourceDataConcept SourceDataType,
          RetryPolicyConcept RetryPolicyType,
          DeliveryStampConcept StampType>
class DefaultDeliveryRequest
{
  public:
    virtual ~DefaultDeliveryRequest() = default;

    DefaultDeliveryRequest()
    {
        static_assert(DeliveryRequestConcept<DefaultDeliveryRequest>, "DefaultDeliveryRequest must satisfy DeliveryRequestConcept");
    }

    using SourceDataType_t = SourceDataType;
    using RetryPolicyType_t = RetryPolicyType;
    using StampType_t = StampType;

    //! Get the source data
    virtual std::shared_ptr<SourceDataType_t> get_source_data() const
    {
        return m_source_data;
    }

    //! Set the source data
    virtual void set_source_data(std::shared_ptr<SourceDataType_t> data)
    {
        m_source_data = data;
    }

    //! Get the stamp
    virtual std::shared_ptr<StampType_t> get_stamp() const
    {
        return m_stamp;
    }

    //! Set the stamp
    virtual void set_stamp(std::shared_ptr<StampType_t> stamp)
    {
        m_stamp = stamp;
    }

    //! Check if this is a ping request
    virtual bool is_ping_request() const
    {
        return m_is_ping;
    }

    //! Set ping flag
    virtual void set_ping_request(bool is_ping)
    {
        m_is_ping = is_ping;
    }

    //! Get the retry policy
    virtual std::shared_ptr<RetryPolicyType_t> get_retry_policy() const
    {
        return m_retry_policy;
    }

    //! Set the retry policy
    virtual void set_retry_policy(std::shared_ptr<RetryPolicyType_t> policy)
    {
        m_retry_policy = policy;
    }

    //! Get the precondition
    virtual DeliveryPrecondition get_precondition() const
    {
        return m_precondition;
    }

    //! Set the precondition
    virtual void set_precondition(DeliveryPrecondition precondition)
    {
        m_precondition = precondition;
    }

    //! Get the drop strategy
    virtual DropStrategy get_drop_strategy() const
    {
        return m_drop_strategy;
    }

    //! Set the drop strategy
    virtual void set_drop_strategy(DropStrategy strategy)
    {
        m_drop_strategy = strategy;
    }

  protected:
    std::shared_ptr<SourceDataType_t> m_source_data;
    std::shared_ptr<StampType_t> m_stamp{std::make_shared<StampType_t>()};
    std::shared_ptr<RetryPolicyType_t> m_retry_policy;
    bool m_is_ping{false};
    DeliveryPrecondition m_precondition{DeliveryPrecondition::NoPrecondition};
    DropStrategy m_drop_strategy{DropStrategy::NoDrop};
};

//! Implementation of DeliveryTaskConcept
template <DeliveryRequestConcept RequestType,
          DeliveryTargetDataConcept TargetDataType,
          RetryPolicyConcept RetryPolicyType>
class DefaultDeliveryTask
{
  public:
    using RequestType_t = RequestType;
    using TargetDataType_t = TargetDataType;
    using RetryPolicyType_t = RetryPolicyType;
    ~DefaultDeliveryTask() = default;

    DefaultDeliveryTask()
    {
        static_assert(DeliveryTaskConcept<DefaultDeliveryTask>, "DefaultDeliveryTask must satisfy DeliveryTaskConcept");
    }

    //! Get the request
    virtual std::shared_ptr<RequestType_t> get_request() const
    {
        return m_request;
    }

    //! Set the request
    virtual void set_request(std::shared_ptr<RequestType_t> request)
    {
        m_request = request;
    }

    //! Get the target data
    virtual std::shared_ptr<TargetDataType_t> get_target_data() const
    {
        return m_target_data;
    }

    //! Set the target data
    virtual void set_target_data(std::shared_ptr<TargetDataType_t> target_data)
    {
        m_target_data = target_data;
    }

    //! Get the retry policy
    virtual std::shared_ptr<RetryPolicyType_t> get_retry_policy() const
    {
        return m_retry_policy;
    }

  protected:
    std::shared_ptr<RequestType_t> m_request;
    std::shared_ptr<TargetDataType_t> m_target_data;
    std::shared_ptr<RetryPolicyType_t> m_retry_policy;
};

//! Default implementation of delivery policy
template <RetryPolicyConcept RetryPolicyType>
class DefaultDeliveryPolicy
{
  public:
    using RetryPolicyType_t = RetryPolicyType;
    ~DefaultDeliveryPolicy() = default;

    DefaultDeliveryPolicy()
    {
        static_assert(DeliveryPolicyConcept<DefaultDeliveryPolicy>, "DefaultDeliveryPolicy must satisfy DeliveryPolicyConcept");
    }

    //! Get the retry policy
    virtual std::shared_ptr<RetryPolicyType_t> get_retry_policy() const
    {
        return m_retry_policy;
    }

    //! Get the precondition
    virtual DeliveryPrecondition get_precondition() const
    {
        return m_precondition;
    }

    //! Get the drop strategy
    virtual DropStrategy get_drop_strategy() const
    {
        return m_drop_strategy;
    }

  protected:
    std::shared_ptr<RetryPolicyType_t> m_retry_policy;
    DeliveryPrecondition m_precondition{DeliveryPrecondition::NoPrecondition};
    DropStrategy m_drop_strategy{DropStrategy::NoDrop};
};

//! Default implementation of downstream specification
template <RosActionConcept ActionType,
          DeliveryPolicyConcept DeliveryPolicyType,
          RosMessageConcept PublishMessageType>
class DefaultDownstreamSpec
{
  public:
    using ActionType_t = ActionType;
    using DeliveryPolicy_t = DeliveryPolicyType;
    using PublishMessageType_t = PublishMessageType;

    virtual ~DefaultDownstreamSpec() = default;

    DefaultDownstreamSpec(const std::string &action_name)
        : m_action_name(action_name)
    {
        static_assert(DownstreamSpecConcept<DefaultDownstreamSpec>, "DefaultDownstreamSpec must satisfy DownstreamSpecConcept");
    }

    //! Get the action name
    virtual const std::string &get_action_name() const
    {
        return m_action_name;
    }

    //! Get the delivery policy
    virtual std::shared_ptr<DeliveryPolicy_t> get_delivery_policy()
    {
        return m_delivery_policy;
    }

    //! Set the delivery policy
    virtual std::shared_ptr<const DeliveryPolicy_t> set_delivery_policy(std::shared_ptr<DeliveryPolicy_t> policy)
    {
        m_delivery_policy = policy;
        return m_delivery_policy;
    }

    //! Get whether to use debug publish
    virtual bool get_use_debug_publish() const
    {
        return m_use_debug_publish;
    }

    //! Get the debug topic for sending
    virtual const std::string *get_debug_topic_sending() const
    {
        return m_use_debug_publish ? &m_debug_topic_sending : nullptr;
    }

    //! Get the debug topic for succeeded
    virtual const std::string *get_debug_topic_succeeded() const
    {
        return m_use_debug_publish ? &m_debug_topic_succeeded : nullptr;
    }

    //! Get the debug topic for failed
    virtual const std::string *get_debug_topic_failed() const
    {
        return m_use_debug_publish ? &m_debug_topic_failed : nullptr;
    }

  protected:
    std::string m_action_name;
    std::shared_ptr<DeliveryPolicy_t> m_delivery_policy;
    bool m_use_debug_publish{false};
    std::string m_debug_topic_sending;
    std::string m_debug_topic_succeeded;
    std::string m_debug_topic_failed;

  public:
    JS_OBJECT(JS_MEMBER_WITH_NAME(m_action_name, "action_name"),
              JS_MEMBER_WITH_NAME(m_use_debug_publish, "use_debug_publish"),
              JS_MEMBER_WITH_NAME(m_debug_topic_sending, "debug_topic_sending"),
              JS_MEMBER_WITH_NAME(m_debug_topic_succeeded, "debug_topic_succeeded"),
              JS_MEMBER_WITH_NAME(m_debug_topic_failed, "debug_topic_failed"));
};

//! Implementation of InitConfigConcept
template <DownstreamSpecConcept TDownstreamSpec>
class DefaultInitConfig
{
  public:
    //! Type aliases
    using DownstreamSpec_t = TDownstreamSpec;

    //! Virtual destructor
    virtual ~DefaultInitConfig() = default;

    //! Get downstream specs (const)
    virtual const std::vector<std::shared_ptr<DownstreamSpec_t>> &get_downstream_specs() const
    {
        return m_downstream_specs;
    }

    //! Get downstream specs (non-const)
    virtual std::vector<std::shared_ptr<DownstreamSpec_t>> &get_downstream_specs()
    {
        return m_downstream_specs;
    }

    //! Get number of buffer requests
    virtual int get_num_buffer_requests() const
    {
        return m_num_buffer_requests;
    }

    //! Get preserve request order flag
    virtual bool get_preserve_request_order() const
    {
        return m_preserve_request_order;
    }

  protected:
    std::vector<std::shared_ptr<DownstreamSpec_t>> m_downstream_specs;
    int m_num_buffer_requests{1};
    bool m_preserve_request_order{true};

  public:
    JS_OBJECT(JS_MEMBER_WITH_NAME(m_num_buffer_requests, "num_buffer_requests"),
              JS_MEMBER_WITH_NAME(m_preserve_request_order, "preserve_request_order"),
              JS_MEMBER_WITH_NAME(m_downstream_specs, "downstream_specs"));
};

//! Implementation of DownstreamConcept
template <DownstreamSpecConcept TDownstreamSpec>
class DefaultDownstream
{
  public:
    //! Type aliases
    using DownstreamSpec_t = TDownstreamSpec;
    using ActionType_t = typename DownstreamSpec_t::ActionType_t;
    using ActionClient_t = rclcpp_action::Client<ActionType_t>;
    using GoalHandle_t = typename ActionClient_t::GoalHandle;
    using SendGoalOptions_t = typename ActionClient_t::SendGoalOptions;
    using PublishMessageType_t = typename DownstreamSpec_t::PublishMessageType_t;
    using PublisherType_t = typename DownstreamSpec_t::PublisherType_t;

    //! Virtual destructor
    virtual ~DefaultDownstream() = default;

    DefaultDownstream()
    {
        static_assert(DownstreamConcept<DefaultDownstream>, "DefaultDownstream must satisfy DownstreamConcept");
    }

    //! Get downstream spec
    virtual std::shared_ptr<DownstreamSpec_t> get_downstream_spec() const
    {
        return m_downstream_spec;
    }

    //! Get action client
    virtual typename ActionClient_t::SharedPtr get_action_client() const
    {
        return m_action_client;
    }

    //! Set action client
    virtual void set_action_client(typename ActionClient_t::SharedPtr client)
    {
        m_action_client = client;
    }

    //! Debug publish to sending topic
    virtual int debug_publish_to_sending_topic(const PublishMessageType_t &data)
    {
        return 0; // Default implementation
    }

    //! Debug publish to succeeded topic
    virtual int debug_publish_to_succeeded_topic(const PublishMessageType_t &data)
    {
        return 0; // Default implementation
    }

    //! Debug publish to failed topic
    virtual int debug_publish_to_failed_topic(const PublishMessageType_t &data)
    {
        return 0; // Default implementation
    }

    //! Initialize downstream from spec
    virtual int init_by_spec(std::shared_ptr<DownstreamSpec_t> spec, rclcpp::Node *node)
    {
        m_downstream_spec = spec;
        m_node = node;

        return 0; // Default implementation
    }

  protected:
    std::shared_ptr<DownstreamSpec_t> m_downstream_spec;
    typename ActionClient_t::SharedPtr m_action_client;
    rclcpp::Node *m_node{nullptr};
};

//! Concept for AsyncActionOutputPortSpec, which is used to define the async action output port
//! @note this is a concept for downstream spec, not the port itself, but the port has to use it
template <typename T>
concept AsyncActionOutputPortSpecConcept = requires(T t)
{
    typename T::ActionType_t;
    requires RosActionConcept<typename T::ActionType_t>;

    typename T::ActionGoal_t;
    requires std::same_as<typename T::ActionGoal_t, typename T::ActionType_t::Goal>;

    typename T::ActionResult_t;
    requires std::same_as<typename T::ActionResult_t, typename T::ActionType_t::Result>;

    typename T::ActionFeedback_t;
    requires std::same_as<typename T::ActionFeedback_t, typename T::ActionType_t::Feedback>;

    typename T::TimeUnit_t;
    requires TimeDurationConcept<typename T::TimeUnit_t>;

    typename T::RetryPolicy_t;
    requires RetryPolicyConcept<typename T::RetryPolicy_t>;
    std::same_as<typename T::RetryPolicy_t::DurationType_t, typename T::TimeUnit_t>;

    typename T::DeliverySourceData_t;
    requires DeliverySourceDataConcept<typename T::DeliverySourceData_t>;

    typename T::SourceDataPublishMessageType_t;
    requires std::same_as<typename T::SourceDataPublishMessageType_t, typename T::DeliverySourceData_t::PublishMessageType_t>;

    typename T::DeliveryTargetData_t;
    requires DeliveryTargetDataConcept<typename T::DeliveryTargetData_t>;
    requires std::same_as<typename T::DeliveryTargetData_t::ActionType_t, typename T::ActionType_t>;
    requires std::same_as<typename T::DeliveryTargetData_t::Goal_t, typename T::ActionGoal_t>;

    typename T::DeliveryStamp_t;
    requires DeliveryStampConcept<typename T::DeliveryStamp_t>;

    typename T::DeliveryRequest_t;
    requires DeliveryRequestConcept<typename T::DeliveryRequest_t>;
    requires std::same_as<typename T::DeliveryRequest_t::SourceDataType_t, typename T::DeliverySourceData_t>;
    requires std::same_as<typename T::DeliveryRequest_t::RetryPolicyType_t, typename T::RetryPolicy_t>;
    requires std::same_as<typename T::DeliveryRequest_t::StampType_t, typename T::DeliveryStamp_t>;

    typename T::DeliveryTask_t;
    requires DeliveryTaskConcept<typename T::DeliveryTask_t>;
    requires std::same_as<typename T::DeliveryTask_t::RequestType_t, typename T::DeliveryRequest_t>;
    requires std::same_as<typename T::DeliveryTask_t::TargetDataType_t, typename T::DeliveryTargetData_t>;
    requires std::same_as<typename T::DeliveryTask_t::RetryPolicyType_t, typename T::RetryPolicy_t>;

    typename T::DeliveryPolicy_t;
    requires DeliveryPolicyConcept<typename T::DeliveryPolicy_t>;
    requires std::same_as<typename T::DeliveryPolicy_t::RetryPolicyType_t, typename T::RetryPolicy_t>;

    typename T::DownstreamSpec_t;
    requires DownstreamSpecConcept<typename T::DownstreamSpec_t>;
    requires std::same_as<typename T::DownstreamSpec_t::ActionType_t, typename T::ActionType_t>;
    requires std::same_as<typename T::DownstreamSpec_t::DeliveryPolicyType_t, typename T::DeliveryPolicy_t>;

    // for debug publishing in downstream nodes
    typename T::DownstreamDebugPublisher_t;
    requires RosPublisherConcept<typename T::DownstreamDebugPublisher_t>;
    requires std::same_as<typename T::DownstreamDebugPublisher_t, typename T::DownstreamSpec_t::PublisherType_t>;
    requires std::same_as<typename T::DownstreamDebugPublisher_t::MessageType_t, typename T::SourceDataPublishMessageType_t>;

    typename T::InitConfig_t;
    requires InitConfigConcept<typename T::InitConfig_t>;
    requires std::same_as<typename T::InitConfig_t::DownstreamSpecType_t, typename T::DownstreamSpec_t>;

    typename T::Downstream_t;
    requires DownstreamConcept<typename T::Downstream_t>;
    requires std::same_as<typename T::Downstream_t::DownstreamSpecType_t, typename T::DownstreamSpec_t>;
};


} // namespace AsyncActionPortTypes

} // namespace redoxi_works
