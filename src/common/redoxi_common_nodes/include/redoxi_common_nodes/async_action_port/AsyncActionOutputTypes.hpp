#pragma once

#include <optional>
#include <typeinfo>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <json_struct/json_struct.h>

#include <redoxi_common_nodes/async_action_port/output_port_concepts.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

// default types for the async action output port
namespace redoxi_works
{

namespace output_port_types
{

//! Sample action type, used to test whether the concepts are working
using _SampleAction = redoxi_public_msgs::action::ProcessFrame;
//! Sample image type, used to test whether the concepts are working
using _SampleImage = sensor_msgs::msg::Image;
//! Sample time unit type, used to test whether the concepts are working
using _TimeUnit = std::chrono::milliseconds;

//! Sample publisher type, used to test whether the concepts are working
struct _SampleSourcePublisher {
    using MessageType_t = _SampleImage;
    int publish(const MessageType_t &)
    {
        return 0;
    }
    int publish(const MessageType_t &, const std::string &)
    {
        return 0;
    }
};
static_assert(RosPublisherConcept<_SampleSourcePublisher>,
              "_SampleSourcePublisher must satisfy RosPublisherConcept");

//! Sample target publisher type, used to test whether the concepts are working
struct _SampleTargetPublisher {
    using MessageType_t = _SampleImage;
    int publish(const MessageType_t &)
    {
        return 0;
    }
    int publish(const MessageType_t &, const std::string &)
    {
        return 0;
    }
};
static_assert(RosPublisherConcept<_SampleTargetPublisher>,
              "_SampleTargetPublisher must satisfy RosPublisherConcept");


//! Sample source data type, used to test whether the concepts are working
struct _SampleSourceData {
    using PublishMessageType_t = _SampleImage;

    _SampleSourceData() = default;

    int to_publish_message(PublishMessageType_t &) const
    {
        return 0;
    }

    boost::uuids::uuid get_uuid() const
    {
        return boost::uuids::uuid();
    }
};
static_assert(DeliverySourceDataConcept<_SampleSourceData>,
              "_SampleSourceData must satisfy DeliverySourceDataConcept");

//! Sample action data trait
using _SampleActionDataTrait = NoneActionDataTrait<_SampleAction>;
static_assert(ActionDataTraitConcept<_SampleActionDataTrait>,
              "_SampleActionDataTrait must satisfy ActionDataTraitConcept");

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

  private: // default fallback values, not visible to the outside
    inline static constexpr int64_t DefaultNumberOfRetry = 3;
    inline static constexpr DurationType_t DefaultWaitTimeBetweenRetry = std::chrono::milliseconds(5);
    inline static constexpr DurationType_t DefaultWaitTimeRetryResponse = std::chrono::milliseconds(100);

  public:
    virtual ~DefaultRetryPolicy() = default;

    //! Get the current number of retry
    virtual std::optional<int64_t> get_number_of_retry(bool use_fallback_if_not_set = false) const
    {
        if (use_fallback_if_not_set && !this->number_of_retry.has_value()) {
            return this->fallback_number_of_retry;
        }
        return this->number_of_retry;
    }

    //! Set the current number of retr
    virtual void set_number_of_retry(std::optional<int64_t> number_of_retry)
    {
        this->number_of_retry = number_of_retry;
    }

    //! Get the fallback number of retry
    virtual int64_t get_fallback_number_of_retry() const
    {
        return fallback_number_of_retry;
    }

    //! Get the current wait time between retries
    virtual std::optional<DurationType_t> get_wait_time_between_retry(bool use_fallback_if_not_set = false) const
    {
        if (use_fallback_if_not_set && !wait_time_between_retry.has_value()) {
            return fallback_wait_time_between_retry;
        }
        return wait_time_between_retry;
    }

    //! Set the current wait time between retries
    virtual void set_wait_time_between_retry(std::optional<DurationType_t> wait_time)
    {
        wait_time_between_retry = wait_time;
    }

    //! Get the fallback wait time between retries
    virtual DurationType_t get_fallback_wait_time_between_retry() const
    {
        return fallback_wait_time_between_retry;
    }

    //! Get the current wait time for retry response
    virtual std::optional<DurationType_t> get_wait_time_retry_response(bool use_fallback_if_not_set = false) const
    {
        if (use_fallback_if_not_set && !wait_time_retry_response.has_value()) {
            return fallback_wait_time_retry_response;
        }
        return wait_time_retry_response;
    }

    //! Set the current wait time for retry response
    virtual void set_wait_time_retry_response(std::optional<DurationType_t> wait_time)
    {
        wait_time_retry_response = wait_time;
    }

    //! Get the fallback wait time for retry response
    virtual DurationType_t get_fallback_wait_time_retry_response() const
    {
        return fallback_wait_time_retry_response;
    }

  protected: // no m_ prefix so that you can use json serialization easier
    //! Current number of retry
    std::optional<int64_t> number_of_retry;
    //! Fallback number of retry
    int64_t fallback_number_of_retry = DefaultNumberOfRetry;
    //! Current wait time between retries
    std::optional<DurationType_t> wait_time_between_retry;
    //! Fallback wait time between retries
    DurationType_t fallback_wait_time_between_retry = DefaultWaitTimeBetweenRetry;
    //! Current wait time for retry response
    std::optional<DurationType_t> wait_time_retry_response;
    //! Fallback wait time for retry response
    DurationType_t fallback_wait_time_retry_response = DefaultWaitTimeRetryResponse;

  public:
    JS_OBJECT(JS_MEMBER(number_of_retry),
              JS_MEMBER(fallback_number_of_retry),
              JS_MEMBER(wait_time_between_retry),
              JS_MEMBER(fallback_wait_time_between_retry),
              JS_MEMBER(wait_time_retry_response),
              JS_MEMBER(fallback_wait_time_retry_response));
};
using _SampleRetryPolicy = DefaultRetryPolicy<_TimeUnit>;
static_assert(RetryPolicyConcept<_SampleRetryPolicy>,
              "_SampleRetryPolicy must satisfy RetryPolicyConcept");


template <RosActionConcept ActionType,
          ActionDataTraitConcept ActionDataTrait,
          RosMessageConcept PublishMessageType>
class DefaultTargetData
{
  public:
    //! The ROS message type that this target data wraps
    using ActionType_t = ActionType;
    using Goal_t = typename ActionType_t::Goal;
    using PublishMessageType_t = PublishMessageType;
    using ActionDataTrait_t = ActionDataTrait;

    virtual ~DefaultTargetData() = default;
    DefaultTargetData(const Goal_t &goal = Goal_t())
    {
        m_goal = goal;
    }

    //! Get the underlying ROS message
    const Goal_t &get_goal() const
    {
        return m_goal;
    }

    //! Get the underlying ROS message
    Goal_t &get_goal()
    {
        return m_goal;
    }

    //! Get the source data UUID
    virtual boost::uuids::uuid get_source_data_uuid() const
    {
        return ActionDataTrait_t::get_uuid(m_goal);
    }

    //! Set the source data UUID
    virtual void set_source_data_uuid(boost::uuids::uuid uuid)
    {
        // mark the goal with the source data UUID
        ActionDataTrait_t::set_uuid(m_goal, uuid);
    }

    //! Get the control signal code
    virtual ControlSignalCode get_control_signal_code() const
    {
        return ActionDataTrait_t::get_control_signal_code(m_goal);
    }

    //! Set the control signal code
    virtual void set_control_signal_code(ControlSignalCode code)
    {
        ActionDataTrait_t::mark_with_control_signal(m_goal, code);
    }

    //! Convert to publish message
    virtual int to_publish_message(PublishMessageType_t &) const
    {
        return 0;
    }

  protected:
    Goal_t m_goal;
};
using _SampleTargetData = DefaultTargetData<_SampleAction, _SampleActionDataTrait, _SampleImage>;
static_assert(DeliveryTargetDataConcept<_SampleTargetData>,
              "_SampleTargetData must satisfy DeliveryTargetDataConcept");

struct DefaultStampData {
};
static_assert(DeliveryStampConcept<DefaultStampData>,
              "DefaultStampData must satisfy DeliveryStampConcept");

//! Default implementation of delivery policy
template <RetryPolicyConcept RetryPolicyType>
class DefaultDeliveryPolicy
{
  public:
    using RetryPolicyType_t = RetryPolicyType;
    virtual ~DefaultDeliveryPolicy() = default;

    //! Get the retry policy
    virtual const RetryPolicyType_t &get_retry_policy() const
    {
        return this->retry_policy;
    }

    //! Get the retry policy
    virtual RetryPolicyType_t &get_retry_policy()
    {
        return this->retry_policy;
    }

    //! Get the precondition
    virtual DeliveryPrecondition get_precondition() const
    {
        return this->precondition;
    }

    //! Get the drop strategy
    virtual DropStrategy get_drop_strategy() const
    {
        return this->drop_strategy;
    }

    //! Set the retry policy
    virtual void set_retry_policy(RetryPolicyType_t policy)
    {
        this->retry_policy = policy;
    }

    //! Set the precondition
    virtual void set_precondition(DeliveryPrecondition precond)
    {
        this->precondition = precond;
    }

    //! Set the drop strategy
    virtual void set_drop_strategy(DropStrategy strategy)
    {
        this->drop_strategy = strategy;
    }

  protected:
    RetryPolicyType_t retry_policy;
    DeliveryPrecondition precondition{DeliveryPrecondition::DontCare};
    DropStrategy drop_strategy{DropStrategy::DontCare};

  public:
    JS_OBJECT(JS_MEMBER(retry_policy),
              JS_MEMBER(precondition),
              JS_MEMBER(drop_strategy));
};
using _SampleDeliveryPolicy = DefaultDeliveryPolicy<_SampleRetryPolicy>;
static_assert(DeliveryPolicyConcept<_SampleDeliveryPolicy>,
              "_SampleDeliveryPolicy must satisfy DeliveryPolicyConcept");

//! Default implementation of DeliveryRequestConcept
template <DeliverySourceDataConcept SourceDataType,
          DeliveryTargetDataConcept TargetDataType,
          DeliveryPolicyConcept DeliveryPolicyType,
          DeliveryStampConcept StampType>
class DefaultDeliveryRequest
{
  public:
    virtual ~DefaultDeliveryRequest() = default;

    using SourceDataType_t = SourceDataType;
    using TargetDataType_t = TargetDataType;
    using DeliveryPolicy_t = DeliveryPolicyType;
    using StampType_t = StampType;
    using TimeUnit_t = typename DeliveryPolicy_t::RetryPolicyType_t::DurationType_t;
    using SendGoalOptions_t = typename rclcpp_action::Client<typename TargetDataType_t::ActionType_t>::SendGoalOptions;

  private:
    inline static constexpr TimeUnit_t DefaultPingResponseWaitTime{std::chrono::milliseconds(50)};

  public:
    //! Get the source data
    virtual const SourceDataType_t &get_source_data() const
    {
        return m_source_data;
    }

    //! Get the source data
    SourceDataType_t &get_source_data()
    {
        return m_source_data;
    }

    //! Set the source data
    virtual void set_source_data(const SourceDataType_t &source_data)
    {
        m_source_data = source_data;
    }

    //! Get the stamp
    virtual const StampType_t &get_stamp() const
    {
        return m_stamp;
    }

    //! Get the stamp
    StampType_t &get_stamp()
    {
        return m_stamp;
    }

    //! Check if this is a ping request
    //! @deprecated Use get_control_signal_code() == ControlSignalCode::Ping instead
    [[deprecated("Use get_control_signal_code() == ControlSignalCode::Ping instead")]] virtual bool is_ping_request() const
    {
        return m_control_signal_code == ControlSignalCode::Ping;
    }

    //! Get the delivery policy, nullptr if not set
    virtual const DeliveryPolicy_t *get_delivery_policy() const
    {
        return m_delivery_policy.has_value() ? &m_delivery_policy.value() : nullptr;
    }
    //! Set the delivery policy, use nullptr to unset
    virtual void set_delivery_policy(const DeliveryPolicy_t &delivery_policy)
    {
        m_delivery_policy = delivery_policy;
    }

    //! Get the delivery policy, nullptr if not set
    DeliveryPolicy_t *get_delivery_policy()
    {
        return m_delivery_policy.has_value() ? &m_delivery_policy.value() : nullptr;
    }
    /**
     * Convert this to a ping request, which is always a no-precondition and can always be dropped.
     * If not so, it will be set. If wait time retry response is not set, it will be set to the default value.
     */
    //! @deprecated Use set_control_signal_code(ControlSignalCode::Ping) instead
    [[deprecated("Use set_control_signal_code(ControlSignalCode::Ping) instead")]] virtual void as_ping()
    {
        set_control_signal_code(ControlSignalCode::Ping);
    }

    //! Convert to target data
    virtual int to_target_data(TargetDataType_t &) const
    {
        // we have default implementation here so that static_assert can be used to check all concepts are satisfied
        throw std::runtime_error("to_target_data() not implemented");
    }

    //! Get the control signal code
    virtual ControlSignalCode get_control_signal_code() const
    {
        return m_control_signal_code;
    }

    //! Set the control signal code. Note that if the code is ping, it will be converted to a ping request,
    //! removing all data in the request.
    virtual void set_control_signal_code(ControlSignalCode code)
    {
        // if ping, convert to ping request
        if (code == ControlSignalCode::Ping) {
            _as_ping();
        } else {
            m_control_signal_code = code;
        }
    }

    //! Send goal options
    SendGoalOptions_t send_goal_options;

  protected:
    //! Source data for the delivery request
    SourceDataType_t m_source_data;

    //! Stamp data for getting delivery in-progress status
    StampType_t m_stamp;

    //! The delivery policy
    std::optional<DeliveryPolicy_t> m_delivery_policy;

    //! Flag indicating if this is a ping request
    ControlSignalCode m_control_signal_code{ControlSignalCode::Normal};

  private:
    //! Convert to a ping request, which is always a no-precondition and can always be dropped.
    //! If not so, it will be set. If wait time retry response is not set, it will be set to the default value.
    void _as_ping()
    {
        // already a ping request
        if (m_control_signal_code == ControlSignalCode::Ping) {
            return;
        }

        // create a new delivery policy if not already set
        if (!m_delivery_policy.has_value()) {
            m_delivery_policy = DeliveryPolicy_t();
        }

        // set the ping response wait time if not already set
        auto &retry_policy = m_delivery_policy.value().get_retry_policy();
        if (!retry_policy.get_wait_time_retry_response().has_value()) {
            retry_policy.set_wait_time_retry_response(DefaultPingResponseWaitTime);
        }

        // ping request has no precondition
        m_delivery_policy.value().set_precondition(DeliveryPrecondition::NoPrecondition);
        m_delivery_policy.value().set_drop_strategy(DropStrategy::DropAsNeeded);

        // set the ping flag
        m_control_signal_code = ControlSignalCode::Ping;
    }
};
using _SampleDeliveryRequest = DefaultDeliveryRequest<_SampleSourceData, _SampleTargetData, _SampleDeliveryPolicy, DefaultStampData>;
static_assert(DeliveryRequestConcept<_SampleDeliveryRequest>,
              "_SampleDeliveryRequest must satisfy DeliveryRequestConcept");

//! Implementation of DeliveryTaskConcept
template <DeliveryRequestConcept RequestType,
          DeliveryTargetDataConcept TargetDataType,
          RetryPolicyConcept RetryPolicyType>
class DefaultDeliveryTask
{
  public:
    using RequestType_t = RequestType;
    using TargetDataType_t = TargetDataType;
    virtual ~DefaultDeliveryTask() = default;

    //! Get the request
    virtual RequestType_t &get_request()
    {
        return m_request;
    }

    //! Get the request (const)
    virtual const RequestType_t &get_request() const
    {
        return m_request;
    }

    //! Set the request
    virtual void set_request(const RequestType_t &request)
    {
        m_request = request;
    }

    //! Get the target data
    virtual TargetDataType_t &get_target_data()
    {
        return m_target_data;
    }

    //! Get the target data (const)
    virtual const TargetDataType_t &get_target_data() const
    {
        return m_target_data;
    }

    //! Set the target data
    virtual void set_target_data(const TargetDataType_t &target_data)
    {
        m_target_data = target_data;
    }

  protected:
    RequestType_t m_request;
    TargetDataType_t m_target_data;
};
using _SampleDeliveryTask = DefaultDeliveryTask<_SampleDeliveryRequest, _SampleTargetData, _SampleRetryPolicy>;
static_assert(DeliveryTaskConcept<_SampleDeliveryTask>,
              "_SampleDeliveryTask must satisfy DeliveryTaskConcept");

//! Default implementation of downstream specification
template <RosActionConcept ActionType,
          DeliveryPolicyConcept DeliveryPolicyType,
          RosPublisherConcept SourceDataPublisherType,
          RosPublisherConcept TargetDataPublisherType>
class DefaultDownstreamSpec
{
  public:
    using ActionType_t = ActionType;
    using DeliveryPolicy_t = DeliveryPolicyType;
    using SourcePublisherType_t = SourceDataPublisherType;
    using TargetPublisherType_t = TargetDataPublisherType;
    using SourcePublishMessageType_t = typename SourcePublisherType_t::MessageType_t;
    using TargetPublishMessageType_t = typename TargetPublisherType_t::MessageType_t;

    virtual ~DefaultDownstreamSpec() = default;


    //! Get the name
    virtual const std::string &get_name() const
    {
        return this->name;
    }

    //! Set the name
    virtual void set_name(const std::string &name)
    {
        this->name = name;
    }

    //! Get the action name
    virtual const std::string &get_action_name() const
    {
        return this->action_name;
    }

    //! Set the action name
    virtual void set_action_name(const std::string &action_name)
    {
        this->action_name = action_name;
    }

    //! Get the delivery policy
    virtual DeliveryPolicy_t &get_delivery_policy()
    {
        return this->delivery_policy;
    }

    //! Get the delivery policy (const)
    virtual const DeliveryPolicy_t &get_delivery_policy() const
    {
        return this->delivery_policy;
    }

    //! Set the delivery policy
    virtual void set_delivery_policy(const DeliveryPolicy_t &policy)
    {
        this->delivery_policy = policy;
    }

    //! Get whether to use debug publish
    virtual bool get_use_debug_publish() const
    {
        return this->create_debug_pub;
    }
    //! Get the debug topic for source data sending
    virtual std::optional<std::string> get_debug_topic_source_data_sending() const
    {
        if (this->debug_topic_source_data_sending.has_value()) {
            return this->debug_topic_source_data_sending;
        }
        return make_debug_topic_name(this->name, "source", "sending");
    }

    //! Set the debug topic for source data sending
    virtual void set_debug_topic_source_data_sending(const std::optional<std::string> &topic)
    {
        this->debug_topic_source_data_sending = topic;
    }

    //! Get the debug topic for source data succeeded
    virtual std::optional<std::string> get_debug_topic_source_data_succeeded() const
    {
        if (this->debug_topic_source_data_succeeded.has_value()) {
            return this->debug_topic_source_data_succeeded;
        }
        return make_debug_topic_name(this->name, "source", "succeeded");
    }

    //! Set the debug topic for source data succeeded
    virtual void set_debug_topic_source_data_succeeded(const std::optional<std::string> &topic)
    {
        this->debug_topic_source_data_succeeded = topic;
    }

    //! Get the debug topic for source data failed
    virtual std::optional<std::string> get_debug_topic_source_data_failed() const
    {
        if (this->debug_topic_source_data_failed.has_value()) {
            return this->debug_topic_source_data_failed;
        }
        return make_debug_topic_name(this->name, "source", "failed");
    }

    //! Set the debug topic for source data failed
    virtual void set_debug_topic_source_data_failed(const std::optional<std::string> &topic)
    {
        this->debug_topic_source_data_failed = topic;
    }

    //! Get the debug topic for target data sending
    virtual std::optional<std::string> get_debug_topic_target_data_sending() const
    {
        if (this->debug_topic_target_data_sending.has_value()) {
            return this->debug_topic_target_data_sending;
        }
        return make_debug_topic_name(this->name, "target", "sending");
    }

    //! Set the debug topic for target data sending
    virtual void set_debug_topic_target_data_sending(const std::optional<std::string> &topic)
    {
        this->debug_topic_target_data_sending = topic;
    }

    //! Get the debug topic for target data succeeded
    virtual std::optional<std::string> get_debug_topic_target_data_succeeded() const
    {
        if (this->debug_topic_target_data_succeeded.has_value()) {
            return this->debug_topic_target_data_succeeded;
        }
        return make_debug_topic_name(this->name, "target", "succeeded");
    }

    //! Set the debug topic for target data succeeded
    virtual void set_debug_topic_target_data_succeeded(const std::optional<std::string> &topic)
    {
        this->debug_topic_target_data_succeeded = topic;
    }

    //! Get the debug topic for target data failed
    virtual std::optional<std::string> get_debug_topic_target_data_failed() const
    {
        if (this->debug_topic_target_data_failed.has_value()) {
            return this->debug_topic_target_data_failed;
        }
        return make_debug_topic_name(this->name, "target", "failed");
    }

    //! Set the debug topic for target data failed
    virtual void set_debug_topic_target_data_failed(const std::optional<std::string> &topic)
    {
        this->debug_topic_target_data_failed = topic;
    }

  protected: // no m_ prefix so that you can use json serialization easier
    static std::string make_debug_topic_name(const std::string &name,
                                             const std::string &data_type,
                                             const std::string &event)
    {
        std::string output = fmt::format("downstream_debug/{}/{}/{}", name, data_type, event);
        // Remove consecutive forward slashes using std::unique
        output.erase(std::unique(output.begin(), output.end(),
                                 [](char a, char b) { return a == '/' && b == '/'; }),
                     output.end());
        return output;
    }

    //! The name of this output port
    std::string name;

    //! The action name
    std::string action_name;

    //! The delivery policy
    DeliveryPolicy_t delivery_policy;

    //! Whether to use debug publish
    bool create_debug_pub{false};

    //! Debug topics for publishing source data events
    std::optional<std::string> debug_topic_source_data_sending;
    std::optional<std::string> debug_topic_source_data_succeeded;
    std::optional<std::string> debug_topic_source_data_failed;

    //! Debug topics for publishing target data events
    std::optional<std::string> debug_topic_target_data_sending;
    std::optional<std::string> debug_topic_target_data_succeeded;
    std::optional<std::string> debug_topic_target_data_failed;

  public:
    JS_OBJECT(JS_MEMBER(name),
              JS_MEMBER(action_name),
              JS_MEMBER(delivery_policy),
              JS_MEMBER(create_debug_pub),
              JS_MEMBER(debug_topic_source_data_sending),
              JS_MEMBER(debug_topic_source_data_succeeded),
              JS_MEMBER(debug_topic_source_data_failed),
              JS_MEMBER(debug_topic_target_data_sending),
              JS_MEMBER(debug_topic_target_data_succeeded),
              JS_MEMBER(debug_topic_target_data_failed));
};
using _SampleDownstreamSpec = DefaultDownstreamSpec<_SampleAction, _SampleDeliveryPolicy,
                                                    _SampleSourcePublisher, _SampleTargetPublisher>;
static_assert(DownstreamSpecConcept<_SampleDownstreamSpec>,
              "_SampleDownstreamSpec must satisfy DownstreamSpecConcept");

//! Implementation of InitConfigConcept
template <DownstreamSpecConcept TDownstreamSpec>
class DefaultInitConfig
{
  public:
    //! Type aliases
    using DownstreamSpec_t = TDownstreamSpec;
    virtual ~DefaultInitConfig() = default;

    //! Get downstream specs (const)
    virtual const std::vector<DownstreamSpec_t> &get_downstream_specs() const
    {
        return this->downstream_specs;
    }

    //! Get downstream specs (non-const)
    virtual std::vector<DownstreamSpec_t> &get_downstream_specs()
    {
        return this->downstream_specs;
    }

    //! Get number of buffer requests
    virtual int get_num_buffer_requests() const
    {
        return this->num_buffer_requests;
    }

    //! Get preserve request order flag
    virtual bool get_preserve_request_order() const
    {
        return this->preserve_request_order;
    }

    //! Set downstream specs
    virtual void set_downstream_specs(const std::vector<DownstreamSpec_t> &specs)
    {
        this->downstream_specs = specs;
    }

    //! Set number of buffer requests
    virtual void set_num_buffer_requests(int num)
    {
        this->num_buffer_requests = num;
    }

    //! Set preserve request order flag
    virtual void set_preserve_request_order(bool preserve)
    {
        this->preserve_request_order = preserve;
    }

    //! Get fallback delivery precondition
    virtual DeliveryPrecondition get_fallback_delivery_precondition() const
    {
        return this->fallback_delivery_precondition;
    }

    //! Set fallback delivery precondition
    virtual void set_fallback_delivery_precondition(DeliveryPrecondition precondition)
    {
        this->fallback_delivery_precondition = precondition;
    }

  protected: // no m_ prefix so that you can use json serialization easier
    std::vector<DownstreamSpec_t> downstream_specs;
    int num_buffer_requests{1};
    bool preserve_request_order{true};
    DeliveryPrecondition fallback_delivery_precondition{DeliveryPrecondition::DontCare};
    std::string _action_goal_type = rosidl_generator_traits::name<typename DownstreamSpec_t::ActionType_t::Goal>();

  public:
    JS_OBJECT(JS_MEMBER(_action_goal_type),
              JS_MEMBER(downstream_specs),
              JS_MEMBER(num_buffer_requests),
              JS_MEMBER(preserve_request_order),
              JS_MEMBER(fallback_delivery_precondition));
};
using _SampleInitConfig = DefaultInitConfig<_SampleDownstreamSpec>;
static_assert(InitConfigConcept<_SampleInitConfig>,
              "_SampleInitConfig must satisfy InitConfigConcept");

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
    using SourcePublishMessageType_t = typename DownstreamSpec_t::SourcePublishMessageType_t;
    using TargetPublishMessageType_t = typename DownstreamSpec_t::TargetPublishMessageType_t;
    using SourcePublisherType_t = typename DownstreamSpec_t::SourcePublisherType_t;
    using TargetPublisherType_t = typename DownstreamSpec_t::TargetPublisherType_t;

    //! Virtual destructor
    virtual ~DefaultDownstream() = default;

    DefaultDownstream()
    {
        static_assert(DownstreamConcept<DefaultDownstream>, "DefaultDownstream must satisfy DownstreamConcept");
    }

    //! Get downstream spec
    virtual const DownstreamSpec_t &get_downstream_spec() const
    {
        return m_downstream_spec;
    }
    DownstreamSpec_t &get_downstream_spec()
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

    //! Get debug publisher for source data sending
    virtual std::shared_ptr<SourcePublisherType_t> get_debug_pub_source_data_sending() const
    {
        return m_debug_pub_source_data_sending;
    }

    //! Get debug publisher for source data succeeded
    virtual std::shared_ptr<SourcePublisherType_t> get_debug_pub_source_data_succeeded() const
    {
        return m_debug_pub_source_data_succeeded;
    }

    //! Get debug publisher for source data failed
    virtual std::shared_ptr<SourcePublisherType_t> get_debug_pub_source_data_failed() const
    {
        return m_debug_pub_source_data_failed;
    }

    //! Get debug publisher for target data sending
    virtual std::shared_ptr<TargetPublisherType_t> get_debug_pub_target_data_sending() const
    {
        return m_debug_pub_target_data_sending;
    }

    //! Get debug publisher for target data succeeded
    virtual std::shared_ptr<TargetPublisherType_t> get_debug_pub_target_data_succeeded() const
    {
        return m_debug_pub_target_data_succeeded;
    }

    //! Get debug publisher for target data failed
    virtual std::shared_ptr<TargetPublisherType_t> get_debug_pub_target_data_failed() const
    {
        return m_debug_pub_target_data_failed;
    }

    //! Initialize downstream from spec
    virtual int init_by_spec(const DownstreamSpec_t &spec, rclcpp::Node *node)
    {
        if (node == nullptr) {
            RDX_RAISE_ERROR("[{}()]: Node is nullptr when initializing downstream with spec '{}'", __func__, spec.get_name());
        }

        m_downstream_spec = spec;
        m_node = node;

        // create action client
        RDX_LOG_DEBUG(node, "[{}] Creating action client for action '{}'", __func__, spec.get_action_name());
        m_action_client = rclcpp_action::create_client<ActionType_t>(node, spec.get_action_name());
        if (m_action_client == nullptr) {
            RDX_RAISE_ERROR("[{}()]: Failed to create action client for action '{}'", __func__, spec.get_action_name());
        }

        bool ret = m_action_client->wait_for_action_server();
        if (!ret) {
            RDX_RAISE_ERROR("[{}()]: Failed to wait for action server for action '{}'", __func__, spec.get_action_name());
        }

        RDX_LOG_DEBUG(node, "[{}] Action client for action '{}' created", __func__, spec.get_action_name());

        return 0;
    }

  protected:
    DownstreamSpec_t m_downstream_spec;
    typename ActionClient_t::SharedPtr m_action_client;
    std::shared_ptr<SourcePublisherType_t> m_debug_pub_source_data_sending;
    std::shared_ptr<SourcePublisherType_t> m_debug_pub_source_data_succeeded;
    std::shared_ptr<SourcePublisherType_t> m_debug_pub_source_data_failed;
    std::shared_ptr<TargetPublisherType_t> m_debug_pub_target_data_sending;
    std::shared_ptr<TargetPublisherType_t> m_debug_pub_target_data_succeeded;
    std::shared_ptr<TargetPublisherType_t> m_debug_pub_target_data_failed;
    rclcpp::Node *m_node{nullptr};
};
using _SampleDownstream = DefaultDownstream<_SampleDownstreamSpec>;
static_assert(DownstreamConcept<_SampleDownstream>,
              "_SampleDownstream must satisfy DownstreamConcept");

//! Concept for AsyncActionOutputPortSpec, which is used to define the async action output port
//! @note this is a concept for downstream spec, not the port itself, but the port has to use it
template <typename T>
concept AsyncActionOutputPortSpecConcept = requires(T t)
{
    //! Action type and related types
    typename T::ActionType_t;
    requires RosActionConcept<typename T::ActionType_t>;

    typename T::ActionGoal_t;
    requires std::same_as<typename T::ActionGoal_t, typename T::ActionType_t::Goal>;

    typename T::ActionResult_t;
    requires std::same_as<typename T::ActionResult_t, typename T::ActionType_t::Result>;

    typename T::ActionFeedback_t;
    requires std::same_as<typename T::ActionFeedback_t, typename T::ActionType_t::Feedback>;

    typename T::ActionDataTrait_t;
    requires ActionDataTraitConcept<typename T::ActionDataTrait_t>;
    requires std::same_as<typename T::ActionDataTrait_t::ActionType_t, typename T::ActionType_t>;

    //! Time unit type
    typename T::TimeUnit_t;
    requires TimeDurationConcept<typename T::TimeUnit_t>;

    //! Retry policy type
    typename T::RetryPolicy_t;
    requires RetryPolicyConcept<typename T::RetryPolicy_t>;
    requires std::same_as<typename T::RetryPolicy_t::DurationType_t, typename T::TimeUnit_t>;

    //! Source data type
    typename T::DeliverySourceData_t;
    requires DeliverySourceDataConcept<typename T::DeliverySourceData_t>;

    //! Source data publish message type
    typename T::SourcePublishMessageType_t;
    requires std::same_as<typename T::SourcePublishMessageType_t, typename T::DeliverySourceData_t::PublishMessageType_t>;

    //! Publisher types for downstream debug publishing
    typename T::SourcePublisherType_t;
    requires RosPublisherConcept<typename T::SourcePublisherType_t>;
    requires std::same_as<typename T::SourcePublisherType_t::MessageType_t, typename T::SourcePublishMessageType_t>;

    //! Target data type
    typename T::DeliveryTargetData_t;
    requires DeliveryTargetDataConcept<typename T::DeliveryTargetData_t>;
    requires std::same_as<typename T::DeliveryTargetData_t::ActionType_t, typename T::ActionType_t>;
    requires std::same_as<typename T::DeliveryTargetData_t::Goal_t, typename T::ActionGoal_t>;

    //! Target data publish message type
    typename T::TargetPublishMessageType_t;
    requires std::same_as<typename T::TargetPublishMessageType_t, typename T::DeliveryTargetData_t::PublishMessageType_t>;

    typename T::TargetPublisherType_t;
    requires RosPublisherConcept<typename T::TargetPublisherType_t>;
    requires std::same_as<typename T::TargetPublisherType_t::MessageType_t, typename T::TargetPublishMessageType_t>;

    //! Stamp type
    typename T::DeliveryStamp_t;
    requires DeliveryStampConcept<typename T::DeliveryStamp_t>;

    //! Delivery policy type
    typename T::DeliveryPolicy_t;
    requires DeliveryPolicyConcept<typename T::DeliveryPolicy_t>;
    requires std::same_as<typename T::DeliveryPolicy_t::RetryPolicyType_t, typename T::RetryPolicy_t>;

    //! Request type
    typename T::DeliveryRequest_t;
    requires DeliveryRequestConcept<typename T::DeliveryRequest_t>;
    requires std::same_as<typename T::DeliveryRequest_t::SourceDataType_t, typename T::DeliverySourceData_t>;
    requires std::same_as<typename T::DeliveryRequest_t::DeliveryPolicy_t, typename T::DeliveryPolicy_t>;
    requires std::same_as<typename T::DeliveryRequest_t::StampType_t, typename T::DeliveryStamp_t>;

    //! Task type
    typename T::DeliveryTask_t;
    requires DeliveryTaskConcept<typename T::DeliveryTask_t>;
    requires std::same_as<typename T::DeliveryTask_t::RequestType_t, typename T::DeliveryRequest_t>;
    requires std::same_as<typename T::DeliveryTask_t::TargetDataType_t, typename T::DeliveryTargetData_t>;

    //! Downstream spec type
    typename T::DownstreamSpec_t;
    requires DownstreamSpecConcept<typename T::DownstreamSpec_t>;
    requires std::same_as<typename T::DownstreamSpec_t::ActionType_t, typename T::ActionType_t>;
    requires std::same_as<typename T::DownstreamSpec_t::DeliveryPolicy_t, typename T::DeliveryPolicy_t>;
    requires std::same_as<typename T::DownstreamSpec_t::SourcePublisherType_t, typename T::SourcePublisherType_t>;
    requires std::same_as<typename T::DownstreamSpec_t::TargetPublisherType_t, typename T::TargetPublisherType_t>;

    //! Config types
    typename T::InitConfig_t;
    requires InitConfigConcept<typename T::InitConfig_t>;
    requires std::same_as<typename T::InitConfig_t::DownstreamSpec_t, typename T::DownstreamSpec_t>;

    typename T::Downstream_t;
    requires DownstreamConcept<typename T::Downstream_t>;
    requires std::same_as<typename T::Downstream_t::DownstreamSpec_t, typename T::DownstreamSpec_t>;
};

//! Implementation of AsyncActionOutputPortSpecConcept
struct _SampleAsyncActionOutputPortSpec {
    //! Action type
    using ActionType_t = _SampleAction;
    using ActionGoal_t = _SampleAction::Goal;
    using ActionResult_t = _SampleAction::Result;
    using ActionFeedback_t = _SampleAction::Feedback;

    //! Time unit type
    using TimeUnit_t = _TimeUnit;

    //! Retry policy type
    using RetryPolicy_t = _SampleRetryPolicy;

    //! Action data trait type
    using ActionDataTrait_t = _SampleActionDataTrait;

    //! Source data type
    using DeliverySourceData_t = _SampleSourceData;
    //! Target data type
    using DeliveryTargetData_t = _SampleTargetData;

    //! Source publisher type
    using SourcePublisherType_t = _SampleSourcePublisher;
    //! Target publisher type
    using TargetPublisherType_t = _SampleTargetPublisher;

    //! Source publish message type
    using SourcePublishMessageType_t = typename SourcePublisherType_t::MessageType_t;
    //! Target publish message type
    using TargetPublishMessageType_t = typename TargetPublisherType_t::MessageType_t;
    //! Delivery policy type
    using DeliveryPolicy_t = _SampleDeliveryPolicy;
    //! Stamp type
    using DeliveryStamp_t = DefaultStampData;
    //! Request type
    using DeliveryRequest_t = _SampleDeliveryRequest;
    //! Task type
    using DeliveryTask_t = _SampleDeliveryTask;
    //! Downstream spec type
    using DownstreamSpec_t = _SampleDownstreamSpec;
    //! Init config type
    using InitConfig_t = _SampleInitConfig;
    //! Downstream type
    using Downstream_t = _SampleDownstream;
};
static_assert(AsyncActionOutputPortSpecConcept<_SampleAsyncActionOutputPortSpec>,
              "_SampleAsyncActionOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");


} // namespace output_port_types

} // namespace redoxi_works
