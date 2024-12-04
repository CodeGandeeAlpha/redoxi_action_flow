#pragma once

#include <any>
#include <boost/uuid/uuid_generators.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <psg_private_msgs/action/process_psg_document.hpp>
#include <psg_master_node/StampedDocumentPub.hpp>


namespace redoxi_works
{

namespace async_action_document_output_port
{
using TimeUnit = DefaultTimeUnit_t;
using DeliveryActionType = psg_private_msgs::action::ProcessPsgDocument;
static_assert(RedoxiActionConcept<DeliveryActionType>, "DeliveryActionType must satisfy RedoxiActionConcept");

namespace Defaults
{
static constexpr int64_t FallbackNumberOfRetry = 3;
static constexpr TimeUnit FallbackWaitTimeBetweenRetry = std::chrono::milliseconds(5);
static constexpr TimeUnit FallbackWaitTimeRetryResponse = std::chrono::milliseconds(1000);
} // namespace Defaults

//! Retry policy type implementing the RetryPolicyConcept
class RetryPolicy : public output_port_types::DefaultRetryPolicy<TimeUnit>
{
  public:
    RetryPolicy()
    {
        fallback_number_of_retry = Defaults::FallbackNumberOfRetry;
        fallback_wait_time_between_retry = Defaults::FallbackWaitTimeBetweenRetry;
        fallback_wait_time_retry_response = Defaults::FallbackWaitTimeRetryResponse;
    }
};

//! Source data type for document output port
//! This type must satisfy the DeliverySourceDataConcept
class DeliverySourceData
{
  public:
    using PublishMessageType_t = psg_private_msgs::msg::PsgDocument;

    DeliverySourceData()
    {
        static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>, "DeliverySourceData must satisfy DeliverySourceDataConcept");
        m_uuid = boost::uuids::random_generator()();
    }
    virtual ~DeliverySourceData() = default;


    //! Get the image
    virtual const PublishMessageType_t &get_document() const
    {
        return m_document;
    }

    //! Set the image
    virtual void set_document(const PublishMessageType_t &document)
    {
        m_document = document;
    }

    //! Convert the source data to a ROS message for publishing
    virtual int to_publish_message(PublishMessageType_t &msg) const
    {
        msg = m_document;
        std::copy(m_uuid.begin(), m_uuid.end(), msg.x_uid.uuid.begin());
        return 0;
    }

    //! Get the UUID associated with this source data
    virtual boost::uuids::uuid get_uuid() const
    {
        return m_uuid;
    }

    //! Set the UUID associated with this source data
    virtual void set_uuid(const boost::uuids::uuid &uuid)
    {
        m_uuid = uuid;
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

  protected:
    boost::uuids::uuid m_uuid;
    psg_private_msgs::msg::PsgDocument m_document;
};


//! Delivery target data type for image output port
using DeliveryTargetDataBase =
    output_port_types::DefaultTargetData<DeliveryActionType, RedoxiActionDataTrait<DeliveryActionType>, DeliverySourceData::PublishMessageType_t>;
class DeliveryTargetData : public DeliveryTargetDataBase
{
  public:
    DeliveryTargetData()
    {
        static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>, "DeliveryTargetData must satisfy DeliveryTargetDataConcept");
    }
    DeliveryTargetData(const Goal_t &goal)
        : DeliveryTargetDataBase(goal)
    {
    }

    virtual int to_publish_message(PublishMessageType_t &msg) const
    {
        msg = this->m_goal.document;
        return 0;
    }

  public:
    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;
};
static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>, "DeliveryTargetData must satisfy DeliveryTargetDataConcept");

//! Stamp data type for image output port (nothing to do here, right now)
using DeliveryStampData = output_port_types::DefaultStampData;

//! Delivery policy type for image output port
using DeliveryPolicy = output_port_types::DefaultDeliveryPolicy<RetryPolicy>;
static_assert(output_port_types::DeliveryPolicyConcept<DeliveryPolicy>, "DeliveryPolicy must satisfy DeliveryPolicyConcept");

//! Request type for image output port
using DeliveryRequestBase = output_port_types::DefaultDeliveryRequest<DeliverySourceData, DeliveryTargetData, DeliveryPolicy, DeliveryStampData>;
class DeliveryRequest : public DeliveryRequestBase
{
  public:
    virtual int to_target_data(DeliveryTargetData &target_data) const
    {
        // apply custom function if set
        if (custom_to_target_data) {
            custom_to_target_data(target_data, *this);
            return 0;
        }

        auto &goal = target_data.get_goal();

        // fill payload
        auto document = this->m_source_data.get_document();

        // convert frame to document
        goal.document = document;

        // set additional information into the goal
        using ActionTrait = DeliveryTargetData::ActionDataTrait_t;

        // set the source data UUID
        ActionTrait::set_uuid(goal, this->m_source_data.get_uuid());

        ActionTrait::mark_with_control_signal(goal, get_control_signal_code());

        return 0;
    }

  public:
    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

    // custom function to transform the request to target data, if set, this will override the default behavior
    std::function<void(DeliveryTargetData &target_data, const DeliveryRequest &request)> custom_to_target_data;
};

static_assert(output_port_types::DeliveryRequestConcept<DeliveryRequest>, "DeliveryRequest must satisfy DeliveryRequestConcept");
using DeliveryTask = output_port_types::DefaultDeliveryTask<DeliveryRequest, DeliveryTargetData, RetryPolicy>;
static_assert(output_port_types::DeliveryTaskConcept<DeliveryTask>, "DeliveryTask must satisfy DeliveryTaskConcept");

//! Downstream debug publisher type for image output port
class DownstreamDebugPublisher
{
  public:
    using MessageType_t = psg_private_msgs::msg::PsgDocument;
    using Publisher_t = StampedDocumentPub;
    inline static const cv::Scalar DefaultHeaderColor{255, 0, 0};
    inline static constexpr double DefaultHeaderScale = 1.0;
    inline static const rclcpp::QoS DefaultQoS = DefaultParams::DebugPublisherQoS;

    //! Constructor for DownstreamDebugPublisher with concept assert
    DownstreamDebugPublisher()
    {
        static_assert(RosPublisherConcept<DownstreamDebugPublisher>,
                      "DownstreamDebugPublisher must satisfy RosPublisherConcept");
    }
    virtual ~DownstreamDebugPublisher() = default;

    //! Initialize the DownstreamDebugPublisher with a shared pointer to a publisher
    virtual void init(std::shared_ptr<Publisher_t> pub)
    {
        m_pub = pub;
    }

    //! Set the publisher for the DownstreamDebugPublisher
    virtual void set_publisher(std::shared_ptr<Publisher_t> pub)
    {
        m_pub = pub;
    }

    //! Get the current publisher of the DownstreamDebugPublisher
    virtual std::shared_ptr<Publisher_t> get_publisher() const
    {
        return m_pub;
    }

    //! Publish an image with the DownstreamDebugPublisher
    virtual int publish(const MessageType_t &msg)
    {
        return m_pub->publish(msg);
    }

    virtual int publish(const MessageType_t &msg,
                        const std::string &header_text)
    {
        return m_pub->publish(msg, header_text);
    }

  protected:
    std::shared_ptr<Publisher_t> m_pub;
};

//! Downstream spec type for image output port
using DownstreamSpec = output_port_types::DefaultDownstreamSpec<
    DeliveryActionType,
    DeliveryPolicy,
    DownstreamDebugPublisher,
    DownstreamDebugPublisher>;
static_assert(output_port_types::DownstreamSpecConcept<DownstreamSpec>,
              "DownstreamSpec must satisfy DefaultDownstreamSpecConcept");

//! Init config type for image output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec>;

//! Downstream type for image output port
using DownstreamBase = output_port_types::DefaultDownstream<DownstreamSpec>;
class Downstream : public DownstreamBase
{
  public:
    Downstream()
    {
        static_assert(output_port_types::DownstreamConcept<Downstream>,
                      "DownstreamSpec must satisfy DefaultDownstreamSpecConcept");
    }

    virtual int init_by_spec(const DownstreamSpec &spec, rclcpp::Node *node)
    {
        auto ret = DownstreamBase::init_by_spec(spec, node);
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}] failed to initialize downstream", __func__);
        }
        auto qos_source = DownstreamSpec::SourcePublisherType_t::DefaultQoS;
        auto qos_target = DownstreamSpec::TargetPublisherType_t::DefaultQoS;
        using SourceInnerPublisherType = DownstreamSpec::SourcePublisherType_t::Publisher_t;
        using TargetInnerPublisherType = DownstreamSpec::TargetPublisherType_t::Publisher_t;

        {
            auto topic = spec.get_debug_topic_source_data_failed();
            if (topic.has_value()) {
                m_debug_pub_source_data_failed = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<SourceInnerPublisherType>(node, topic.value(), qos_source);
                m_debug_pub_source_data_failed->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_source_data_sending();
            if (topic.has_value()) {
                m_debug_pub_source_data_sending = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<SourceInnerPublisherType>(node, topic.value(), qos_source);
                m_debug_pub_source_data_sending->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_source_data_succeeded();
            if (topic.has_value()) {
                m_debug_pub_source_data_succeeded = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<SourceInnerPublisherType>(node, topic.value(), qos_source);
                m_debug_pub_source_data_succeeded->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_target_data_sending();
            if (topic.has_value()) {
                m_debug_pub_target_data_sending = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<TargetInnerPublisherType>(node, topic.value(), qos_target);
                m_debug_pub_target_data_sending->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_target_data_succeeded();
            if (topic.has_value()) {
                m_debug_pub_target_data_succeeded = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<TargetInnerPublisherType>(node, topic.value(), qos_target);
                m_debug_pub_target_data_succeeded->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_target_data_failed();
            if (topic.has_value()) {
                m_debug_pub_target_data_failed = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<TargetInnerPublisherType>(node, topic.value(), qos_target);
                m_debug_pub_target_data_failed->init(pub);
            }
        }

        return 0;
    }
};

//! document output port spec
//! This type must satisfy the AsyncActionOutputPortSpecConcept
//! Any async output port can use this spec as a template argument
struct PSGDocumentOutputPortSpec {
    PSGDocumentOutputPortSpec()
    {
        static_assert(output_port_types::AsyncActionOutputPortSpecConcept<PSGDocumentOutputPortSpec>,
                      "PSGDocumentOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
    }
    //! Action type and related types
    using ActionType_t = DeliveryActionType;
    using ActionGoal_t = typename ActionType_t::Goal;
    using ActionResult_t = typename ActionType_t::Result;
    using ActionFeedback_t = typename ActionType_t::Feedback;

    // the action type trait
    using ActionDataTrait_t = RedoxiActionDataTrait<ActionType_t>;

    //! Time unit type
    using TimeUnit_t = TimeUnit;

    //! Retry policy type
    using RetryPolicy_t = RetryPolicy;

    //! Source data type
    using DeliverySourceData_t = DeliverySourceData;

    //! Source data publish message type
    using SourcePublishMessageType_t = typename DeliverySourceData_t::PublishMessageType_t;

    //! Source data publisher type
    using SourcePublisherType_t = DownstreamDebugPublisher;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data publish message type
    using TargetPublishMessageType_t = typename DeliveryTargetData_t::PublishMessageType_t;

    //! Target data publisher type
    using TargetPublisherType_t = DownstreamDebugPublisher;

    //! Stamp type
    using DeliveryStamp_t = output_port_types::DefaultStampData;

    //! Request type
    using DeliveryRequest_t = DeliveryRequest;

    //! Task type
    using DeliveryTask_t = DeliveryTask;

    //! Delivery policy type
    using DeliveryPolicy_t = DeliveryPolicy;

    //! Downstream spec type
    using DownstreamSpec_t = DownstreamSpec;

    //! Init config type
    using InitConfig_t = InitConfig;

    //! Downstream type
    using Downstream_t = Downstream;
};

} // namespace async_action_document_output_port

} // namespace redoxi_works