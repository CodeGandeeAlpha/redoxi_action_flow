#pragma once

#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>

namespace redoxi_works
{

namespace async_action_image_output_port
{

namespace aapt = AsyncActionPortTypes;
using TimeUnit = DefaultTimeUnit_t;

namespace Defaults
{
static constexpr int64_t FallbackNumberOfRetry = 3;
static constexpr TimeUnit FallbackWaitTimeBetweenRetry = std::chrono::milliseconds(5);
static constexpr TimeUnit FallbackWaitTimeRetryResponse = std::chrono::milliseconds(1000);
} // namespace Defaults

class RetryPolicy : public aapt::DefaultRetryPolicy<TimeUnit>
{
    RetryPolicy()
    {
        m_fallback_number_of_retry = Defaults::FallbackNumberOfRetry;
        m_fallback_wait_time_between_retry = Defaults::FallbackWaitTimeBetweenRetry;
        m_fallback_wait_time_retry_response = Defaults::FallbackWaitTimeRetryResponse;
    }
};

//! Source data type for image output port
class ImageSourceData
{
  public:
    using PublishMessageType_t = sensor_msgs::msg::Image;

    //! Get the image data
    virtual const cv::Mat &get_image() const
    {
        return m_image;
    }

    //! Set the image data
    virtual void set_image(const cv::Mat &image)
    {
        m_image = image;
    }

  protected:
    cv::Mat m_image;
};

//! Image output port spec
struct ImageOutputPortSpec {
    //! Action type
    using ActionType_t = TAction;
    using ActionGoal_t = typename ActionType_t::Goal;
    using ActionResult_t = typename ActionType_t::Result;
    using ActionFeedback_t = typename ActionType_t::Feedback;

    //! Time unit type
    using TimeUnit_t = rclcpp::Duration;

    //! Retry policy type
    using RetryPolicy_t = aapt::DefaultRetryPolicy<TimeUnit_t>;

    //! Source data type
    using DeliverySourceData_t = ImageSourceData;
    using SourceDataPublishMessageType_t = typename DeliverySourceData_t::PublishMessageType_t;

    //! Target data type
    using DeliveryTargetData_t = aapt::DefaultTargetData<ActionType_t>;

    //! Stamp type
    using DeliveryStamp_t = aapt::DefaultStampData;

    //! Request type
    using DeliveryRequest_t = aapt::DefaultDeliveryRequest<
        DeliverySourceData_t,
        RetryPolicy_t,
        DeliveryStamp_t>;

    //! Task type
    using DeliveryTask_t = aapt::DefaultDeliveryTask<
        DeliveryRequest_t,
        DeliveryTargetData_t,
        RetryPolicy_t>;

    //! Delivery policy type
    using DeliveryPolicy_t = aapt::DefaultDeliveryPolicy<RetryPolicy_t>;

    //! Publisher type for debug publishing
    using DownstreamDebugPublisher_t = redoxi_works::StampedImagePub;

    //! Downstream spec type
    using DownstreamSpec_t = aapt::DefaultDownstreamSpec<
        ActionType_t,
        DeliveryPolicy_t,
        SourceDataPublishMessageType_t>;

    //! Init config type
    using InitConfig_t = aapt::DefaultInitConfig<DownstreamSpec_t>;

    //! Downstream type
    using Downstream_t = aapt::DefaultDownstream<DownstreamSpec_t>;

    static_assert(aapt::AsyncActionOutputPortSpecConcept<ImageOutputPortSpec>,
                  "ImageOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
};

} // namespace async_action_image_output_port

} // namespace redoxi_works