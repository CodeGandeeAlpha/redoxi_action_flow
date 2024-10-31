#pragma once

#include <redoxi_common_nodes/async_action_port/output_port_concepts.hpp>
#include <optional>
#include <json_struct/json_struct.h>

#include <redoxi_public_msgs/action/process_frame.hpp>
#include <sensor_msgs/msg/image.hpp>

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
struct _SampleActionDataTrait {
    using ActionType_t = _SampleAction;
    using Goal_t = typename ActionType_t::Goal;
    using Result_t = typename ActionType_t::Result;
    using Feedback_t = typename ActionType_t::Feedback;

    static ControlSignalCode get_control_signal_code(const Goal_t &)
    {
        return ControlSignalCode::Normal;
    }

    static void mark_with_control_signal(Goal_t &, ControlSignalCode)
    {
    }

    static boost::uuids::uuid get_uuid(const Goal_t &)
    {
        return boost::uuids::uuid();
    }

    static void set_uuid(Goal_t &, boost::uuids::uuid)
    {
    }
};
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
    inline static constexpr DurationType_t DefaultWaitTimeBetweenRetry = std::chrono::milliseconds(10);
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

    virtual DefaultTargetData &operator=(const DefaultTargetData &) = default;

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
        return m_source_data_uuid;
    }

    //! Set the source data UUID
    virtual void set_source_data_uuid(boost::uuids::uuid uuid)
    {
        m_source_data_uuid = uuid;
    }

    //! Convert to publish message
    virtual int to_publish_message(PublishMessageType_t &) const
    {
        return 0;
    }

  protected:
    Goal_t m_goal;
    boost::uuids::uuid m_source_data_uuid;
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
    DeliveryPrecondition precondition{DeliveryPrecondition::NoPrecondition};
    DropStrategy drop_strategy{DropStrategy::NoDrop};

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
          DeliveryPolicyConcept DeliveryPolicyType,
          DeliveryStampConcept StampType>
class DefaultDeliveryRequest
{
  public:
    virtual ~DefaultDeliveryRequest() = default;

    using SourceDataType_t = SourceDataType;
    using DeliveryPolicy_t = DeliveryPolicyType;
    using StampType_t = StampType;
    using TimeUnit_t = typename DeliveryPolicy_t::RetryPolicyType_t::DurationType_t;

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
    virtual bool is_ping_request() const
    {
        return m_is_ping;
    }

    //! Get the delivery policy
    virtual const DeliveryPolicy_t &get_delivery_policy() const
    {
        return m_delivery_policy;
    }
    //! Set the delivery policy
    virtual void set_delivery_policy(const DeliveryPolicy_t &delivery_policy)
    {
        m_delivery_policy = delivery_policy;
    }

    //! Get the delivery policy
    DeliveryPolicy_t &get_delivery_policy()
    {
        return m_delivery_policy;
    }
    /**
     * Convert this to a ping request, which is always a no-precondition and can always be dropped.
     * If not so, it will be set. If wait time retry response is not set, it will be set to the default value.
     */
    virtual void as_ping()
    {
        // already a ping request
        if (m_is_ping) {
            return;
        }

        // set the ping response wait time if not already set
        auto &retry_policy = m_delivery_policy.get_retry_policy();
        if (!retry_policy.get_wait_time_retry_response().has_value()) {
            retry_policy.set_wait_time_retry_response(DefaultPingResponseWaitTime);
        }

        // ping request has no precondition
        m_delivery_policy.set_precondition(DeliveryPrecondition::NoPrecondition);
        m_delivery_policy.set_drop_strategy(DropStrategy::DropAsNeeded);

        // set the ping flag
        m_is_ping = true;
    }

  protected:
    //! Source data for the delivery request
    SourceDataType_t m_source_data;

    //! Stamp data for getting delivery in-progress status
    StampType_t m_stamp;

    //! The delivery policy
    DeliveryPolicy_t m_delivery_policy;

    //! Flag indicating if this is a ping request
    bool m_is_ping{false};
};
using _SampleDeliveryRequest = DefaultDeliveryRequest<_SampleSourceData, _SampleDeliveryPolicy, DefaultStampData>;
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
    //! Initialize the downstream spec
    virtual void init(const std::string &name, const std::string &action_name)
    {
        this->name = name;
        this->action_name = action_name;
        this->debug_topic_source_data_sending = "debug/" + name + "/source_data/sending";
        this->debug_topic_source_data_succeeded = "debug/" + name + "/source_data/succeeded";
        this->debug_topic_source_data_failed = "debug/" + name + "/source_data/failed";
        this->debug_topic_target_data_sending = "debug/" + name + "/target_data/sending";
        this->debug_topic_target_data_succeeded = "debug/" + name + "/target_data/succeeded";
        this->debug_topic_target_data_failed = "debug/" + name + "/target_data/failed";
    }

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
        return this->use_debug_publish;
    }

    //! Get the debug topic for source data sending
    virtual std::optional<std::string> get_debug_topic_source_data_sending() const
    {
        return this->debug_topic_source_data_sending;
    }

    //! Get the debug topic for source data succeeded
    virtual std::optional<std::string> get_debug_topic_source_data_succeeded() const
    {
        return this->debug_topic_source_data_succeeded;
    }

    //! Get the debug topic for source data failed
    virtual std::optional<std::string> get_debug_topic_source_data_failed() const
    {
        return this->debug_topic_source_data_failed;
    }

    //! Get the debug topic for target data sending
    virtual std::optional<std::string> get_debug_topic_target_data_sending() const
    {
        return this->debug_topic_target_data_sending;
    }

    //! Get the debug topic for target data succeeded
    virtual std::optional<std::string> get_debug_topic_target_data_succeeded() const
    {
        return this->debug_topic_target_data_succeeded;
    }

    //! Get the debug topic for target data failed
    virtual std::optional<std::string> get_debug_topic_target_data_failed() const
    {
        return this->debug_topic_target_data_failed;
    }

  protected: // no m_ prefix so that you can use json serialization easier
    //! The name of this output port
    std::string name;

    //! The action name
    std::string action_name;

    //! The delivery policy
    DeliveryPolicy_t delivery_policy;

    //! Whether to use debug publish
    bool use_debug_publish{false};

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
              JS_MEMBER(use_debug_publish),
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

  protected: // no m_ prefix so that you can use json serialization easier
    std::vector<DownstreamSpec_t> downstream_specs;
    int num_buffer_requests{1};
    bool preserve_request_order{true};

  public:
    JS_OBJECT(JS_MEMBER(downstream_specs),
              JS_MEMBER(num_buffer_requests),
              JS_MEMBER(preserve_request_order));
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
        m_downstream_spec = spec;
        m_node = node;

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
