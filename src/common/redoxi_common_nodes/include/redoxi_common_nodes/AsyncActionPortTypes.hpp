#pragma once

#include "redoxi_common_nodes/redoxi_common_nodes.hpp"
#include <redoxi_common_cpp/interfaces/IDeliveryRetryPolicy.hpp>
#include <string>
#include <optional>
#include <vector>

#include <rclcpp_action/client.hpp>
#include <opencv2/opencv.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <boost/uuid/uuid.hpp>

namespace redoxi_works
{

//! Concept to check if a type is ROS action goal
template <typename T>
concept RosActionGoalConcept = requires
{
    //! Check Goal type exists and is accessible
    typename T::Goal;
    //! Check Result type exists and is accessible
    typename T::Result;
    //! Check Feedback type exists and is accessible
    typename T::Feedback;

    //! Check Impl struct exists and has required service/message types
    typename T::Impl::SendGoalService;
    typename T::Impl::GetResultService;
    typename T::Impl::FeedbackMessage;
    typename T::Impl::CancelGoalService;
    typename T::Impl::GoalStatusMessage;
};

//! Interface for retry policy, if anything needs to retry, its configuration should be here
class IRetryPolicy
{
  public:
    virtual ~IRetryPolicy() = default;

    //! Get the maximum number of retries allowed
    //! @return The maximum number of retries, or std::nullopt if unlimited
    virtual std::optional<int64_t> get_number_of_retry() const = 0;

    //! Set the maximum number of retries allowed
    //! @param number_of_retry The maximum number of retries, or std::nullopt for unlimited
    virtual void set_number_of_retry(std::optional<int64_t> number_of_retry) = 0;

    //! Get the wait time between retry attempts
    //! @return The wait time between retries, or std::nullopt if no wait time
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_between_retry() const = 0;

    //! Set the wait time between retry attempts
    //! @param wait_time The wait time between retries, or std::nullopt for no wait
    virtual void set_wait_time_between_retry(std::optional<DefaultTimeUnit_t> wait_time) = 0;

    //! Get the maximum wait time for a retry response
    //! @return The maximum wait time for response, or std::nullopt if no timeout
    virtual std::optional<DefaultTimeUnit_t> get_wait_time_retry_response() const = 0;

    //! Set the maximum wait time for a retry response
    //! @param wait_time The maximum wait time for response, or std::nullopt for no timeout
    virtual void set_wait_time_retry_response(std::optional<DefaultTimeUnit_t> wait_time) = 0;
};

enum class DeliveryPrecondition {
    //! No precondition, just deliver
    NoPrecondition = 0,

    //! Any downstream must be ready
    AnyDownstreamReady = 1,

    //! All downstreams must be ready
    AllDownstreamsReady = 2,
};

enum class DropStrategy {
    //! Do not drop
    NoDrop = 0,

    //! Drop task/data/messages as needed
    DropAsNeeded = 1,
};

namespace AsyncActionPortTypes
{
//! The type of the goal for the downstream action
using DownstreamGoalType = redoxi_public_msgs::action::ProcessFrame;
using PublishDataType = sensor_msgs::msg::Image;

//! data to be sent to the downstream action, in its original format
//! for example, a cv::Mat image
struct DeliverySourceData {
    cv::Mat image;
};

//! data to be sent to the downstream action, in a format that the downstream action can use
//! for example, a ROS message
struct DeliveryTargetData {
    sensor_msgs::msg::Image image;
};

//! data collected during the delivery process
struct DeliveryStamp {
};

//! The request to deliver to the downstream action
struct DeliveryRequest {
    //! Get the source data, can be nullptr for a ping request
    std::shared_ptr<DeliverySourceData> get_source_data() const;

    //! Get the stamp where the delivery process can write data to
    std::shared_ptr<DeliveryStamp> get_stamp() const;

    //! is this a ping request?
    bool is_ping_request() const;

    //! get/set retry policy
    std::shared_ptr<IRetryPolicy> get_retry_policy() const;

    //! get/set precondition
    DeliveryPrecondition get_precondition() const;
    void set_precondition(DeliveryPrecondition precondition);

    //! get/set drop strategy
    DropStrategy get_drop_strategy() const;
    void set_drop_strategy(DropStrategy drop_strategy);
};


//! A task to deliver to the downstream action
struct DeliveryTask {
    //! get/set request data
    std::shared_ptr<DeliveryRequest> get_request() const;
    void set_request(std::shared_ptr<DeliveryRequest> request);

    //! get/set target data
    void set_target_data(std::shared_ptr<DeliveryTargetData> target_data);
    std::shared_ptr<DeliveryTargetData> get_target_data() const;

    //! get/set retry policy
    std::shared_ptr<IRetryPolicy> get_retry_policy() const;
    void set_retry_policy(std::shared_ptr<IRetryPolicy> retry_policy);

    //! get/set precondition
    DeliveryPrecondition get_precondition() const;
    void set_precondition(DeliveryPrecondition precondition);

    //! get/set drop strategy
    DropStrategy get_drop_strategy() const;
    void set_drop_strategy(DropStrategy drop_strategy);
};

//! The delivery policy for sending the downstream action
struct DeliveryPolicy {
    //! Retry policy for sending attempts
    std::shared_ptr<IRetryPolicy> get_retry_policy() const;

    //! The precondition for delivery
    DeliveryPrecondition get_precondition() const;

    //! The strategy for dropping messages
    DropStrategy get_drop_strategy() const;
};

// template <RosActionGoalConcept GoalType = DownstreamGoalType>
struct DownstreamSpec {
    // the types must be copyable
    using ActionType_t = DownstreamGoalType;
    using DeliveryPolicy_t = DeliveryPolicy;
    using PublishDataType_t = PublishDataType;
    //! Get the action name
    const std::string &get_action_name() const;

    //! Get the delivery policy
    std::shared_ptr<DeliveryPolicy_t> get_delivery_policy() const;

    // use debug publish?
    bool get_use_debug_publish() const;

    // debug publish topic, nullptr if not used
    const std::string *get_debug_topic_sending() const;   // during delivery attempts are made
    const std::string *get_debug_topic_succeeded() const; // after a delivery attempt succeeds
    const std::string *get_debug_topic_failed() const;    // after a delivery attempt fails
};

struct InitConfig {
    //! The downstream specs
    const std::vector<std::shared_ptr<DownstreamSpec>> &get_downstream_specs() const;
    std::vector<std::shared_ptr<DownstreamSpec>> &get_downstream_specs();
};

struct Downstream {
    using DownstreamSpec_t = AsyncActionPortTypes::DownstreamSpec;
    using ActionType_t = typename DownstreamSpec_t::ActionType_t;
    using ActionClient_t = rclcpp_action::Client<ActionType_t>;
    using GoalHandle_t = typename ActionClient_t::GoalHandle;
    using SendGoalOptions_t = typename ActionClient_t::SendGoalOptions;
    using PublishDataType_t = typename DownstreamSpec_t::PublishDataType_t;

    // the downstream spec
    std::shared_ptr<DownstreamSpec_t> get_downstream_spec() const;

    // the downstream action client
    std::shared_ptr<ActionClient_t> get_action_client() const;

    // publish topics
    int debug_publish_to_sending_topic(const PublishDataType_t &data);
    int debug_publish_to_succeeded_topic(const PublishDataType_t &data);
    int debug_publish_to_failed_topic(const PublishDataType_t &data);
};


} // namespace AsyncActionPortTypes

} // namespace redoxi_works