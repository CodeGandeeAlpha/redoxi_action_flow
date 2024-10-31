#pragma once

#include <redoxi_common_nodes/common_concepts.hpp>

namespace redoxi_works
{

namespace output_port_types
{

//! data to be sent to the downstream action, in its original format
//! for example, a cv::Mat image, which is not related to ROS
template <typename T>
concept DeliverySourceDataConcept = requires(T t)
{
    requires std::copyable<T>;
    requires std::is_default_constructible_v<T>;

    //! The source data can be converted to this type for publishing
    typename T::PublishMessageType_t;
    requires RosMessageConcept<typename T::PublishMessageType_t>;

    //! Must have method to get ROS message
    {
        std::declval<const T &>().to_publish_message(std::declval<typename T::PublishMessageType_t &>())
        } -> std::same_as<int>;

    //! Must have method to get UUID
    {
        std::declval<const T &>().get_uuid()
        } -> std::same_as<boost::uuids::uuid>;
};

//! data to be sent to the downstream action, in a format that the downstream action can use
//! for example, a ROS message
template <typename T>
concept DeliveryTargetDataConcept = requires(T t)
{
    //! Must have ROS message type
    typename T::ActionType_t;
    requires RosActionConcept<typename T::ActionType_t>;

    typename T::Goal_t;
    requires std::same_as<typename T::Goal_t, typename T::ActionType_t::Goal>;

    //! Must have publish message type, can be used for debug publishing
    typename T::PublishMessageType_t;
    requires RosMessageConcept<typename T::PublishMessageType_t>;

    //! Must have action data trait
    typename T::ActionDataTrait_t;
    requires ActionDataTraitConcept<typename T::ActionDataTrait_t>;

    //! Must be copyable
    requires std::copyable<T>;

    //! Must be constructible with no parameters
    requires std::is_default_constructible_v<T>;

    //! Must be constructible with a goal
    requires std::constructible_from<T, const typename T::Goal_t &>;

    //! Must have method to get ROS message
    {
        std::declval<const T &>().get_goal()
        } -> std::same_as<const typename T::Goal_t &>;

    {
        t.get_goal()
        } -> std::same_as<typename T::Goal_t &>;

    //! Must have method to copy data to another target data object
    {
        std::declval<const T &>().copy_to(std::declval<T &>())
        } -> std::same_as<void>;

    //! Must have method to save/read source data UUID
    {
        std::declval<const T &>().get_source_data_uuid()
        } -> std::same_as<boost::uuids::uuid>;
    {
        std::declval<T &>().set_source_data_uuid(std::declval<boost::uuids::uuid>())
        } -> std::same_as<void>;

    //! Must have method to convert to publish message
    {
        std::declval<const T &>().to_publish_message(std::declval<typename T::PublishMessageType_t &>())
        } -> std::same_as<int>;
};

//! data collected during the delivery process
template <typename T>
concept DeliveryStampConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;
    requires std::copyable<T>;
};


//! Concept for delivery policy that defines how to send downstream actions
template <typename T>
concept DeliveryPolicyConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;

    //! Must have these type aliases
    typename T::RetryPolicyType_t;

    //! Retry policy type must satisfy RetryPolicyConcept
    requires RetryPolicyConcept<typename T::RetryPolicyType_t>;

    //! Must have method to get retry policy
    {
        t.get_retry_policy()
        } -> std::same_as<std::shared_ptr<typename T::RetryPolicyType_t>>;

    //! Must have method to get precondition
    {
        t.get_precondition()
        } -> std::same_as<DeliveryPrecondition>;

    //! Must have method to get drop strategy
    {
        t.get_drop_strategy()
        } -> std::same_as<DropStrategy>;
};

//! The request to deliver to the downstream action
template <typename T>
concept DeliveryRequestConcept = requires(T t)
{
    //! Must have these type aliases
    typename T::SourceDataType_t;
    typename T::DeliveryPolicy_t;
    typename T::StampType_t;

    //! Source data type must satisfy DeliverySourceDataConcept
    requires DeliverySourceDataConcept<typename T::SourceDataType_t>;
    //! Delivery policy type must satisfy DeliveryPolicyConcept
    requires DeliveryPolicyConcept<typename T::DeliveryPolicy_t>;
    //! Stamp type must satisfy DeliveryStampConcept
    requires DeliveryStampConcept<typename T::StampType_t>;

    //! Required default constructor
    requires std::is_default_constructible_v<T>;

    //! Required methods
    {
        t.get_source_data()
        } -> std::same_as<std::shared_ptr<typename T::SourceDataType_t>>;
    {
        t.get_stamp()
        } -> std::same_as<std::shared_ptr<typename T::StampType_t>>;
    {
        t.is_ping_request()
        } -> std::same_as<bool>;
    {
        t.get_delivery_policy()
        } -> std::same_as<std::shared_ptr<typename T::DeliveryPolicy_t>>;

    //! Must have method to convert this to a ping request
    //! To create a ping request, you do T().as_ping()
    {
        t.as_ping()
        } -> std::same_as<void>;
};

//! A task to deliver to the downstream action
template <typename T>
concept DeliveryTaskConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;

    //! Must have these type aliases
    typename T::RequestType_t;
    typename T::TargetDataType_t;

    //! Request type must satisfy DeliveryRequestConcept
    requires DeliveryRequestConcept<typename T::RequestType_t>;
    //! Target data type must satisfy DeliveryTargetDataConcept
    requires DeliveryTargetDataConcept<typename T::TargetDataType_t>;

    //! Required methods
    {
        t.get_request()
        } -> std::same_as<std::shared_ptr<typename T::RequestType_t>>;
    {
        t.set_request(std::declval<std::shared_ptr<typename T::RequestType_t>>())
        } -> std::same_as<void>;
    {
        t.get_target_data()
        } -> std::same_as<std::shared_ptr<typename T::TargetDataType_t>>;
    {
        t.set_target_data(std::declval<std::shared_ptr<typename T::TargetDataType_t>>())
        } -> std::same_as<void>;
};


//! Concept for downstream specification that defines how to send downstream actions
template <typename T>
concept DownstreamSpecConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;

    //! Must have action type
    typename T::ActionType_t;
    //! Must have delivery policy type
    typename T::DeliveryPolicy_t;

    //! Must have publisher type for source data
    typename T::SourcePublisherType_t;
    //! Must have publisher type for target data
    typename T::TargetPublisherType_t;
    //! Must have publish message type for source data
    typename T::SourcePublishMessageType_t;
    //! Must have publish message type for target data
    typename T::TargetPublishMessageType_t;

    //! Message types must match publisher message types
    requires RosPublisherConcept<typename T::SourcePublisherType_t>;
    requires RosPublisherConcept<typename T::TargetPublisherType_t>;
    requires std::same_as<typename T::SourcePublishMessageType_t, typename T::SourcePublisherType_t::MessageType_t>;
    requires std::same_as<typename T::TargetPublishMessageType_t, typename T::TargetPublisherType_t::MessageType_t>;

    //! Delivery policy type must satisfy DeliveryPolicyConcept
    requires DeliveryPolicyConcept<typename T::DeliveryPolicy_t>;
    //! Action type must satisfy RosActionConcept
    requires RosActionConcept<typename T::ActionType_t>;

    //! Must have method to get its own name
    {
        std::declval<const T &>().get_name()
        } -> std::same_as<const std::string &>;

    //! Must have method to set its own name
    {
        std::declval<T &>().set_name(std::declval<const std::string &>())
        } -> std::same_as<void>;

    //! Must have method to get action name
    {
        std::declval<const T &>().get_action_name()
        } -> std::same_as<const std::string &>;

    //! Must have method to get delivery policy
    {
        std::declval<T &>().get_delivery_policy()
        } -> std::same_as<std::shared_ptr<typename T::DeliveryPolicy_t>>;
    {
        std::declval<T &>().set_delivery_policy(std::declval<std::shared_ptr<typename T::DeliveryPolicy_t>>())
        } -> std::same_as<std::shared_ptr<const typename T::DeliveryPolicy_t>>;

    //! Must have method to get debug publish flag
    {
        std::declval<const T &>().get_use_debug_publish()
        } -> std::same_as<bool>;

    //! Must have methods to get source data debug topics
    {
        std::declval<const T &>().get_debug_topic_source_data_sending()
        } -> std::same_as<std::optional<std::string>>;

    {
        std::declval<const T &>().get_debug_topic_source_data_succeeded()
        } -> std::same_as<std::optional<std::string>>;

    {
        std::declval<const T &>().get_debug_topic_source_data_failed()
        } -> std::same_as<std::optional<std::string>>;

    //! Must have methods to get target data debug topics
    {
        std::declval<const T &>().get_debug_topic_target_data_sending()
        } -> std::same_as<std::optional<std::string>>;

    {
        std::declval<const T &>().get_debug_topic_target_data_succeeded()
        } -> std::same_as<std::optional<std::string>>;

    {
        std::declval<const T &>().get_debug_topic_target_data_failed()
        } -> std::same_as<std::optional<std::string>>;
};

//! Concept for initialization configuration
template <typename T>
concept InitConfigConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;

    typename T::DownstreamSpec_t;
    requires DownstreamSpecConcept<typename T::DownstreamSpec_t>;

    //! Must have both const and non-const methods to get downstream specs
    {
        std::declval<const T &>().get_downstream_specs()
        } -> std::same_as<const std::vector<std::shared_ptr<typename T::DownstreamSpec_t>> &>;

    {
        std::declval<T &>().get_downstream_specs()
        } -> std::same_as<std::vector<std::shared_ptr<typename T::DownstreamSpec_t>> &>;

    //! Must have method to get number of buffer requests
    {
        std::declval<const T &>().get_num_buffer_requests()
        } -> std::same_as<int>;

    //! Must have method to get preserve request order flag
    {
        std::declval<const T &>().get_preserve_request_order()
        } -> std::same_as<bool>;
};

//! Concept for downstream interface
template <typename T>
concept DownstreamConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;

    typename T::DownstreamSpec_t;
    requires DownstreamSpecConcept<typename T::DownstreamSpec_t>;

    typename T::ActionType_t;
    requires std::same_as<typename T::ActionType_t, typename T::DownstreamSpec_t::ActionType_t>;

    typename T::ActionClient_t;
    requires std::same_as<typename T::ActionClient_t, rclcpp_action::Client<typename T::ActionType_t>>;

    typename T::GoalHandle_t;
    requires std::same_as<typename T::GoalHandle_t, typename T::ActionClient_t::GoalHandle>;

    typename T::SendGoalOptions_t;
    requires std::same_as<typename T::SendGoalOptions_t, typename T::ActionClient_t::SendGoalOptions>;

    //! Source data publisher type
    typename T::SourcePublisherType_t;
    requires RosPublisherConcept<typename T::SourcePublisherType_t>;
    requires std::same_as<typename T::SourcePublisherType_t,
                          typename T::DownstreamSpec_t::SourcePublisherType_t>;

    //! Source data publish message type
    typename T::SourcePublishMessageType_t;
    requires RosMessageConcept<typename T::SourcePublishMessageType_t>;
    requires std::same_as<typename T::SourcePublishMessageType_t,
                          typename T::DownstreamSpec_t::SourcePublishMessageType_t>;

    //! Target data publisher type
    typename T::TargetPublisherType_t;
    requires RosPublisherConcept<typename T::TargetPublisherType_t>;
    requires std::same_as<typename T::TargetPublisherType_t,
                          typename T::DownstreamSpec_t::TargetPublisherType_t>;

    //! Target data publish message type
    typename T::TargetPublishMessageType_t;
    requires RosMessageConcept<typename T::TargetPublishMessageType_t>;
    requires std::same_as<typename T::TargetPublishMessageType_t,
                          typename T::DownstreamSpec_t::TargetPublishMessageType_t>;

    //! Must have method to get downstream spec
    {
        std::declval<const T &>().get_downstream_spec()
        } -> std::same_as<std::shared_ptr<typename T::DownstreamSpec_t>>;

    //! Must have methods to get/set action client
    {
        std::declval<const T &>().get_action_client()
        } -> std::same_as<typename T::ActionClient_t::SharedPtr>;

    {
        std::declval<T &>().set_action_client(std::declval<typename T::ActionClient_t::SharedPtr>())
        } -> std::same_as<void>;

    //! Must have initialization method
    {
        std::declval<T &>().init_by_spec(std::declval<std::shared_ptr<typename T::DownstreamSpec_t>>(), std::declval<rclcpp::Node *>())
        } -> std::same_as<int>;

    //! Get source data debug publishers
    {
        std::declval<const T &>().get_debug_pub_source_data_sending()
        } -> std::same_as<std::shared_ptr<typename T::SourcePublisherType_t>>;

    {
        std::declval<const T &>().get_debug_pub_source_data_succeeded()
        } -> std::same_as<std::shared_ptr<typename T::SourcePublisherType_t>>;

    {
        std::declval<const T &>().get_debug_pub_source_data_failed()
        } -> std::same_as<std::shared_ptr<typename T::SourcePublisherType_t>>;

    //! Get target data debug publishers
    {
        std::declval<const T &>().get_debug_pub_target_data_sending()
        } -> std::same_as<std::shared_ptr<typename T::TargetPublisherType_t>>;

    {
        std::declval<const T &>().get_debug_pub_target_data_succeeded()
        } -> std::same_as<std::shared_ptr<typename T::TargetPublisherType_t>>;

    {
        std::declval<const T &>().get_debug_pub_target_data_failed()
        } -> std::same_as<std::shared_ptr<typename T::TargetPublisherType_t>>;
};


} // namespace output_port_types

} // namespace redoxi_works