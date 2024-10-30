#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace redoxi_works
{

namespace async_action_image_output_port
{
using TimeUnit = DefaultTimeUnit_t;
using DeliveryActionType = redoxi_public_msgs::action::ProcessFrame;

namespace Defaults
{
static constexpr int64_t FallbackNumberOfRetry = 3;
static constexpr TimeUnit FallbackWaitTimeBetweenRetry = std::chrono::milliseconds(5);
static constexpr TimeUnit FallbackWaitTimeRetryResponse = std::chrono::milliseconds(1000);
} // namespace Defaults

//! Retry policy type implementing the RetryPolicyConcept
class RetryPolicy : public output_port_types::DefaultRetryPolicy<TimeUnit>
{
    RetryPolicy()
    {
        m_fallback_number_of_retry = Defaults::FallbackNumberOfRetry;
        m_fallback_wait_time_between_retry = Defaults::FallbackWaitTimeBetweenRetry;
        m_fallback_wait_time_retry_response = Defaults::FallbackWaitTimeRetryResponse;
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
        // Generate or retrieve a UUID for this image data
        // This is a placeholder for actual UUID generation logic
        return boost::uuids::random_generator()();
    }

  protected:
    cv::Mat m_image;
    std::string m_encoding = DefaultEncoding;
    boost::uuids::uuid m_uuid;
};


//! Delivery target data type for image output port
using DeliveryTargetData = output_port_types::DefaultTargetData<DeliveryActionType>;

//! Stamp data type for image output port (nothing to do here, right now)
using DeliveryStampData = output_port_types::DefaultStampData;

//! Request type for image output port
using DeliveryRequest = output_port_types::DefaultDeliveryRequest<DeliverySourceData, RetryPolicy, DeliveryStampData>;

//! Task type for image output port
using DeliveryTask = output_port_types::DefaultDeliveryTask<DeliveryRequest, DeliveryTargetData, RetryPolicy>;

//! Delivery policy type for image output port
using DeliveryPolicy = output_port_types::DefaultDeliveryPolicy<RetryPolicy>;

//! Downstream debug publisher type for image output port
class DownstreamDebugPublisher
{
  public:
    using MessageType_t = sensor_msgs::msg::Image;
    using Publisher_t = redoxi_works::StampedImagePub;
    inline static const cv::Scalar DefaultHeaderColor{255, 0, 0};
    inline static constexpr double DefaultHeaderScale = 1.0;

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
    DownstreamDebugPublisher>;

//! Init config type for image output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec>;

//! Downstream type for image output port
using Downstream = output_port_types::DefaultDownstream<DownstreamSpec>;

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

    //! Time unit type
    using TimeUnit_t = TimeUnit;

    //! Retry policy type
    using RetryPolicy_t = RetryPolicy;

    //! Source data type
    using DeliverySourceData_t = DeliverySourceData;

    //! Source data publish message type
    using SourceDataPublishMessageType_t = typename DeliverySourceData_t::PublishMessageType_t;

    //! Target data type
    using DeliveryTargetData_t = output_port_types::DefaultTargetData<ActionType_t>;

    //! Stamp type
    using DeliveryStamp_t = output_port_types::DefaultStampData;

    //! Request type
    using DeliveryRequest_t = DeliveryRequest;

    //! Task type
    using DeliveryTask_t = DeliveryTask;

    //! Delivery policy type
    using DeliveryPolicy_t = DeliveryPolicy;

    //! Publisher type for debug publishing
    using DownstreamDebugPublisher_t = DownstreamDebugPublisher;

    //! Downstream spec type
    using DownstreamSpec_t = DownstreamSpec;

    //! Init config type
    using InitConfig_t = InitConfig;

    //! Downstream type
    using Downstream_t = Downstream;
};

} // namespace async_action_image_output_port

} // namespace redoxi_works