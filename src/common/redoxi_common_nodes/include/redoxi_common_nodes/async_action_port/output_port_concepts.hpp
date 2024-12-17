#pragma once

#include <redoxi_common_cpp/common_concepts.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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

    //! The source data can be converted to this type for visualization
    typename T::PubVisualizationMsgType_t;
    requires RosMessageConcept<typename T::PubVisualizationMsgType_t>;

    //! The source data can be converted to this type for unreliable data transmission
    typename T::PubDataMsgType_t;
    requires RosMessageConcept<typename T::PubDataMsgType_t>;

    //! Convert to visualization message
    {
        std::declval<const T &>().to_publish_visualization(std::declval<typename T::PubVisualizationMsgType_t &>())
        } -> std::same_as<int>;

    //! Convert to data message for unreliable data transmission
    {
        std::declval<const T &>().to_publish_data(std::declval<typename T::PubDataMsgType_t &>())
        } -> std::same_as<int>;

    //! Must have method to get UUID
    {
        std::declval<const T &>().get_uuid()
        } -> std::same_as<UUIDType>;
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
    typename T::PubVisualizationMsgType_t;
    requires RosMessageConcept<typename T::PubVisualizationMsgType_t>;

    //! Must have publish message type for lossy data transmission
    typename T::PubDataMsgType_t;
    requires RosMessageConcept<typename T::PubDataMsgType_t>;

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

    //! Must have method to save/read source data UUID
    {
        std::declval<const T &>().get_source_data_uuid()
        } -> std::same_as<UUIDType>;
    {
        std::declval<T &>().set_source_data_uuid(std::declval<UUIDType>())
        } -> std::same_as<void>;

    //! Must have method to get the control signal code
    {
        std::declval<const T &>().get_control_signal_code()
        } -> std::same_as<ControlSignalCode>;
    {
        std::declval<T &>().set_control_signal_code(std::declval<ControlSignalCode>())
        } -> std::same_as<void>;

    //! Must have method to convert to publish message
    {
        std::declval<const T &>().to_publish_visualization(std::declval<typename T::PubVisualizationMsgType_t &>())
        } -> std::same_as<int>;

    //! Must have method to convert to unreliable data transmission message
    {
        std::declval<const T &>().to_publish_data(std::declval<typename T::PubDataMsgType_t &>())
        } -> std::same_as<int>;

    //! Get/set task metadata
    {
        std::declval<const T &>().get_source_task_metadata()
        } -> std::same_as<RosActionTaskMetadata>;
    {
        std::declval<T &>().set_source_task_metadata(std::declval<const RosActionTaskMetadata &>())
        } -> std::same_as<void>;
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
        } -> std::same_as<typename T::RetryPolicyType_t &>;

    {
        std::declval<const T &>().get_retry_policy()
        } -> std::same_as<const typename T::RetryPolicyType_t &>;

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
    typename T::TargetDataType_t;
    typename T::DeliveryPolicy_t;
    typename T::StampType_t;
    typename T::SendGoalOptions_t;

    //! Source data type must satisfy DeliverySourceDataConcept
    requires DeliverySourceDataConcept<typename T::SourceDataType_t>;

    //! Target data type must satisfy DeliveryTargetDataConcept
    requires DeliveryTargetDataConcept<typename T::TargetDataType_t>;

    //! Delivery policy type must satisfy DeliveryPolicyConcept
    requires DeliveryPolicyConcept<typename T::DeliveryPolicy_t>;

    //! Stamp type must satisfy DeliveryStampConcept
    requires DeliveryStampConcept<typename T::StampType_t>;

    //! Send goal options type must satisfy SendGoalOptionsConcept
    requires std::same_as<typename T::SendGoalOptions_t,
                          typename rclcpp_action::Client<typename T::TargetDataType_t::ActionType_t>::SendGoalOptions>;

    //! Required default constructor
    requires std::is_default_constructible_v<T>;

    //! Required methods
    {
        t.get_source_data()
        } -> std::same_as<typename T::SourceDataType_t &>;
    {
        std::declval<const T &>().get_source_data()
        } -> std::same_as<const typename T::SourceDataType_t &>;
    {
        t.get_stamp()
        } -> std::same_as<typename T::StampType_t &>;
    {
        std::declval<const T &>().get_stamp()
        } -> std::same_as<const typename T::StampType_t &>;
    {
        t.get_delivery_policy()
        } -> std::same_as<typename T::DeliveryPolicy_t *>;
    {
        std::declval<const T &>().get_delivery_policy()
        } -> std::same_as<const typename T::DeliveryPolicy_t *>;


    //! must be able to convert to target data
    //! @return 0 if success, -1 if failed
    {
        std::declval<const T &>().to_target_data(std::declval<typename T::TargetDataType_t &>())
        } -> std::same_as<int>;

    //! Must have method to get and set control signal code
    {
        std::declval<const T &>().get_control_signal_code()
        } -> std::same_as<ControlSignalCode>;
    {
        std::declval<T &>().set_control_signal_code(std::declval<ControlSignalCode>())
        } -> std::same_as<void>;

    //! Must have method to get and set send goal options
    {
        std::declval<const T &>().send_goal_options
        } -> std::same_as<const typename T::SendGoalOptions_t &>;
    {
        std::declval<T &>().send_goal_options
        } -> std::same_as<typename T::SendGoalOptions_t &>;

    //! get/set source task metadata
    {
        std::declval<const T &>().get_source_task_metadata()
        } -> std::same_as<const RosActionTaskMetadata &>;
    {
        std::declval<T &>().set_source_task_metadata(std::declval<const RosActionTaskMetadata &>())
        } -> std::same_as<void>;
};

//! A task to deliver to the downstream action
template <typename T>
concept DeliveryTaskConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;
    requires std::copyable<T>;

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
        } -> std::same_as<typename T::RequestType_t &>;
    {
        std::declval<const T &>().get_request()
        } -> std::same_as<const typename T::RequestType_t &>;
    {
        t.set_request(std::declval<const typename T::RequestType_t &>())
        } -> std::same_as<void>;
    {
        t.get_target_data()
        } -> std::same_as<typename T::TargetDataType_t &>;
    {
        std::declval<const T &>().get_target_data()
        } -> std::same_as<const typename T::TargetDataType_t &>;
    {
        t.set_target_data(std::declval<const typename T::TargetDataType_t &>())
        } -> std::same_as<void>;
};


//! Concept for downstream specification that defines how to send downstream actions
template <typename T>
concept DownstreamSpecConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;
    requires std::copyable<T>;
    //! Must have action type
    typename T::ActionType_t;
    //! Must have delivery policy type
    typename T::DeliveryPolicy_t;

    //! Must have publisher and message type for source data visualization
    typename T::SourceVisualizationPublisher_t;
    typename T::SourcePubVisualizationMsgType_t;
    requires RosPublisherConcept<typename T::SourceVisualizationPublisher_t>;
    requires std::same_as<typename T::SourcePubVisualizationMsgType_t, typename T::SourceVisualizationPublisher_t::MessageType_t>;

    //! Must have publisher and message type for target data visualization
    typename T::TargetVisualizationPublisher_t;
    typename T::TargetPubVisualizationMsgType_t;
    requires RosPublisherConcept<typename T::TargetVisualizationPublisher_t>;
    requires std::same_as<typename T::TargetPubVisualizationMsgType_t, typename T::TargetVisualizationPublisher_t::MessageType_t>;

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
        } -> std::same_as<typename T::DeliveryPolicy_t &>;
    {
        std::declval<const T &>().get_delivery_policy()
        } -> std::same_as<const typename T::DeliveryPolicy_t &>;
    {
        std::declval<T &>().set_delivery_policy(std::declval<const typename T::DeliveryPolicy_t &>())
        } -> std::same_as<void>;

    //! Must have method to get debug publish flag
    {
        std::declval<const T &>().get_use_debug_publish()
        } -> std::same_as<bool>;

    //! Must have methods to get source data debug topics
    {
        std::declval<const T &>().get_vis_topic_source_data_sending()
        } -> std::same_as<std::optional<std::string>>;

    {
        std::declval<const T &>().get_vis_topic_source_data_succeeded()
        } -> std::same_as<std::optional<std::string>>;

    {
        std::declval<const T &>().get_vis_topic_source_data_failed()
        } -> std::same_as<std::optional<std::string>>;

    //! Must have methods to get target data debug topics
    {
        std::declval<const T &>().get_vis_topic_target_data_sending()
        } -> std::same_as<std::optional<std::string>>;

    {
        std::declval<const T &>().get_vis_topic_target_data_succeeded()
        } -> std::same_as<std::optional<std::string>>;

    {
        std::declval<const T &>().get_vis_topic_target_data_failed()
        } -> std::same_as<std::optional<std::string>>;
};

//! Concept for initialization configuration
template <typename T>
concept InitConfigConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;
    requires std::copyable<T>;

    typename T::DownstreamSpec_t;
    requires DownstreamSpecConcept<typename T::DownstreamSpec_t>;

    //! Must have source data publisher, for publishing data outside
    typename T::SourceDataPublisher_t;
    typename T::SourcePubDataMsgType_t;
    requires RosPublisherConcept<typename T::SourceDataPublisher_t>;
    requires std::same_as<typename T::SourcePubDataMsgType_t, typename T::SourceDataPublisher_t::MessageType_t>;

    //! Must have target data publisher, for publishing data outside
    typename T::TargetDataPublisher_t;
    typename T::TargetPubDataMsgType_t;
    requires RosPublisherConcept<typename T::TargetDataPublisher_t>;
    requires std::same_as<typename T::TargetPubDataMsgType_t, typename T::TargetDataPublisher_t::MessageType_t>;

    //! Must have both const and non-const methods to get downstream specs
    {
        std::declval<const T &>().get_downstream_specs()
        } -> std::same_as<const std::vector<typename T::DownstreamSpec_t> &>;

    {
        std::declval<T &>().get_downstream_specs()
        } -> std::same_as<std::vector<typename T::DownstreamSpec_t> &>;

    //! Must have method to get number of buffer requests
    {
        std::declval<const T &>().get_num_buffer_requests()
        } -> std::convertible_to<int64_t>;

    //! Must have method to get preserve request order flag
    {
        std::declval<const T &>().get_preserve_request_order()
        } -> std::same_as<bool>;

    //! Must have method to get fallback delivery precondition
    {
        std::declval<const T &>().get_fallback_delivery_precondition()
        } -> std::same_as<DeliveryPrecondition>;

    //! Must have method to get source data publish topic
    {
        std::declval<const T &>().get_data_topic_for_source_data()
        } -> std::same_as<std::optional<std::string>>;

    //! Must have method to get source visualization topic
    {
        std::declval<const T &>().get_visualization_topic_for_source_data()
        } -> std::same_as<std::optional<std::string>>;

    //! Must have method to get target data publish topic
    {
        std::declval<const T &>().get_data_topic_for_target_data()
        } -> std::same_as<std::optional<std::string>>;

    //! Must have method to get target visualization topic
    {
        std::declval<const T &>().get_visualization_topic_for_target_data()
        } -> std::same_as<std::optional<std::string>>;
};

//! Concept for downstream interface
template <typename T>
concept DownstreamConcept = requires(T t)
{
    requires std::is_default_constructible_v<T>;
    requires std::copyable<T>;

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
    typename T::SourceVisualizationPublisher_t;
    requires RosPublisherConcept<typename T::SourceVisualizationPublisher_t>;
    requires std::same_as<typename T::SourceVisualizationPublisher_t,
                          typename T::DownstreamSpec_t::SourceVisualizationPublisher_t>;

    //! Source data publish message type
    typename T::SourcePubVisualizationMsgType_t;
    requires RosMessageConcept<typename T::SourcePubVisualizationMsgType_t>;
    requires std::same_as<typename T::SourcePubVisualizationMsgType_t,
                          typename T::DownstreamSpec_t::SourcePubVisualizationMsgType_t>;

    //! Target data publisher type
    typename T::TargetVisualizationPublisher_t;
    requires RosPublisherConcept<typename T::TargetVisualizationPublisher_t>;
    requires std::same_as<typename T::TargetVisualizationPublisher_t,
                          typename T::DownstreamSpec_t::TargetVisualizationPublisher_t>;

    //! Target data publish message type
    typename T::TargetPubVisualizationMsgType_t;
    requires RosMessageConcept<typename T::TargetPubVisualizationMsgType_t>;
    requires std::same_as<typename T::TargetPubVisualizationMsgType_t,
                          typename T::DownstreamSpec_t::TargetPubVisualizationMsgType_t>;

    //! Must have method to get downstream spec
    {
        std::declval<const T &>().get_downstream_spec()
        } -> std::same_as<const typename T::DownstreamSpec_t &>;

    {
        std::declval<T &>().get_downstream_spec()
        } -> std::same_as<typename T::DownstreamSpec_t &>;

    //! Must have methods to get/set action client
    {
        std::declval<const T &>().get_action_client()
        } -> std::same_as<typename T::ActionClient_t::SharedPtr>;

    {
        std::declval<T &>().set_action_client(std::declval<typename T::ActionClient_t::SharedPtr>())
        } -> std::same_as<void>;

    //! Must have initialization method
    {
        std::declval<T &>().init_by_spec(
            std::declval<const typename T::DownstreamSpec_t &>(),
            std::declval<rclcpp::Node *>())
        } -> std::same_as<int>;

    //! Get source data debug publishers
    {
        std::declval<const T &>().get_debug_pub_source_data_sending()
        } -> std::same_as<std::shared_ptr<typename T::SourceVisualizationPublisher_t>>;

    {
        std::declval<const T &>().get_debug_pub_source_data_succeeded()
        } -> std::same_as<std::shared_ptr<typename T::SourceVisualizationPublisher_t>>;

    {
        std::declval<const T &>().get_debug_pub_source_data_failed()
        } -> std::same_as<std::shared_ptr<typename T::SourceVisualizationPublisher_t>>;

    //! Get target data debug publishers
    {
        std::declval<const T &>().get_debug_pub_target_data_sending()
        } -> std::same_as<std::shared_ptr<typename T::TargetVisualizationPublisher_t>>;

    {
        std::declval<const T &>().get_debug_pub_target_data_succeeded()
        } -> std::same_as<std::shared_ptr<typename T::TargetVisualizationPublisher_t>>;

    {
        std::declval<const T &>().get_debug_pub_target_data_failed()
        } -> std::same_as<std::shared_ptr<typename T::TargetVisualizationPublisher_t>>;
};


} // namespace output_port_types

} // namespace redoxi_works