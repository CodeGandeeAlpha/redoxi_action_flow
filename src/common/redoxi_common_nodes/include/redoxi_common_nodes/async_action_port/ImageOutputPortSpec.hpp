#pragma once

#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <nlohmann/json.hpp>

namespace redoxi_works
{

namespace async_action_image_output_port
{
using TimeUnit = DefaultTimeUnit_t;
using DeliveryActionType = redoxi_public_msgs::action::ProcessFrame;
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

//! Source data type for image output port
//! This type must satisfy the DeliverySourceDataConcept
class DeliverySourceData
{
  public:
    using PublishMessageType_t = sensor_msgs::msg::Image;
    inline static constexpr const char *DefaultEncoding = "bgr8";

    DeliverySourceData()
    {
        static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>, "DeliverySourceData must satisfy DeliverySourceDataConcept");
        m_uuid = boost::uuids::random_generator()();
    }
    virtual ~DeliverySourceData() = default;

    //! Get the encoding of the image
    virtual const std::string &get_encoding() const
    {
        return m_encoding;
    }

    //! Set the encoding of the image
    virtual void set_encoding(const std::string &encoding)
    {
        m_encoding = encoding;
    }

    //! Get the image
    virtual const cv::Mat &get_image() const
    {
        return m_image;
    }

    //! Set the image
    virtual void set_image(const cv::Mat &image)
    {
        m_image = image;
    }

    //! Get the frame number
    virtual int64_t get_frame_number() const
    {
        return m_frame_number;
    }

    //! Set the frame number
    virtual void set_frame_number(int64_t frame_number)
    {
        m_frame_number = frame_number;
    }

    //! Convert the source data to a ROS message for publishing
    virtual int to_publish_message(PublishMessageType_t &msg) const
    {
        cv_bridge::CvImage cv_image;
        cv_image.image = m_image;
        cv_image.encoding = m_encoding;
        cv_image.toImageMsg(msg);
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

  protected:
    cv::Mat m_image;
    std::string m_encoding = DefaultEncoding;
    boost::uuids::uuid m_uuid;
    int64_t m_frame_number = 0;
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
        auto &raw_image = this->m_goal.frame.raw_image;
        if (!raw_image.data.empty()) {
            msg = raw_image;
        }
        return 0;
    }
};
static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>, "DeliveryTargetData must satisfy DeliveryTargetDataConcept");

//! Stamp data type for image output port (nothing to do here, right now)
using DeliveryStampData = output_port_types::DefaultStampData;

//! Delivery policy type for image output port
using DeliveryPolicy = output_port_types::DefaultDeliveryPolicy<RetryPolicy>;
static_assert(output_port_types::DeliveryPolicyConcept<DeliveryPolicy>, "DeliveryPolicy must satisfy DeliveryPolicyConcept");

//! Request type for image output port
using DeliveryRequestBase = output_port_types::DefaultDeliveryRequest<DeliverySourceData, DeliveryTargetData, DeliveryPolicy, DeliveryStampData>;

static_assert(output_port_types::DeliveryRequestConcept<DeliveryRequestBase>, "DeliveryRequest must satisfy DeliveryRequestConcept");
class DeliveryRequest : public DeliveryRequestBase
{
  public:
    virtual int to_target_data(DeliveryTargetData &target_data) const
    {
        auto &goal = target_data.get_goal();

        // fill payload
        auto image = this->m_source_data.get_image();

        // convert image to ROS message
        cv_bridge::CvImage cv_bridge_image;
        cv_bridge_image.image = image;
        cv_bridge_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_bridge_image.toImageMsg(goal.frame.raw_image);
        goal.frame.frame_num = this->m_source_data.get_frame_number();

        // set additional information into the goal
        using ActionTrait = DeliveryTargetData::ActionDataTrait_t;

        // set the source data UUID
        ActionTrait::set_uuid(goal, this->m_source_data.get_uuid());

        // set the control signal code
        if (this->is_ping_request()) {
            ActionTrait::mark_with_control_signal(goal, ControlSignalCode::Ping);
        }

        return 0;
    }
};
using DeliveryTask = output_port_types::DefaultDeliveryTask<DeliveryRequest, DeliveryTargetData, RetryPolicy>;
static_assert(output_port_types::DeliveryTaskConcept<DeliveryTask>, "DeliveryTask must satisfy DeliveryTaskConcept");

//! Downstream debug publisher type for image output port
class DownstreamDebugPublisher
{
  public:
    using MessageType_t = sensor_msgs::msg::Image;
    using Publisher_t = redoxi_works::StampedImagePub;
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
    virtual void init(std::shared_ptr<Publisher_t> pub,
                      std::optional<cv::Scalar> header_color = std::nullopt,
                      std::optional<double> header_scale = std::nullopt)
    {
        m_pub = pub;
        m_header_color = header_color.value_or(DefaultHeaderColor);
        m_header_scale = header_scale.value_or(DefaultHeaderScale);
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
    virtual int publish(const cv::Mat &image)
    {
        return m_pub->publish(image);
    }

    virtual int publish(const MessageType_t &msg)
    {
        return m_pub->publish(msg);
    }

    virtual int publish(const MessageType_t &msg,
                        const std::string &header_text)
    {
        return m_pub->publish(msg, header_text, m_header_color, m_header_scale);
    }

  protected:
    std::shared_ptr<Publisher_t> m_pub;
    cv::Scalar m_header_color{DefaultHeaderColor};
    double m_header_scale = DefaultHeaderScale;
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

//! Image output port spec
//! This type must satisfy the AsyncActionOutputPortSpecConcept
//! Any async output port can use this spec as a template argument
struct ImageOutputPortSpec {
    ImageOutputPortSpec()
    {
        static_assert(output_port_types::AsyncActionOutputPortSpecConcept<ImageOutputPortSpec>,
                      "ImageOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
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

} // namespace async_action_image_output_port

} // namespace redoxi_works