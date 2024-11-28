#pragma once

#include <any>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionRequestCommon.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <redoxi_common_nodes/image_ports/ImageOutputPortSpec.hpp>

namespace redoxi_works::detection_ports::request_response::types
{

//! Retry policy type implementing the RetryPolicyConcept
using RetryPolicy = output_port_types::DefaultRetryPolicy<TimeUnit>;

//! Source data type for detection request output port
using DeliverySourceData = image_ports::types::DeliverySourceData;
// class DeliverySourceData
// {
//   public:
//     using PublishMessageType_t = sensor_msgs::msg::Image;
//     using ActionDataTrait_t = DetectionRequestActionDataTrait;
//     inline static constexpr const char *DefaultPublishEncoding = sensor_msgs::image_encodings::BGR8;

//     DeliverySourceData()
//     {
//         static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>,
//                       "DeliverySourceData must satisfy DeliverySourceDataConcept");
//         uid = boost::uuids::random_generator()();
//     }

//     virtual ~DeliverySourceData() = default;

//     //! Get the UUID associated with this source data
//     virtual boost::uuids::uuid get_uuid() const
//     {
//         return uid;
//     }

//     //! Convert the source data to a ROS message for publishing
//     virtual int to_publish_message(PublishMessageType_t &msg) const
//     {
//         if (!image.has_value() || image->empty()) {
//             // no image to publish
//             return -1;
//         }

//         cv::Mat output_image = image->clone();

//         //! Draw detections on the image
//         for (const auto &detection : detections) {
//             // Draw bounding box
//             cv::Rect bbox(detection.bbox.x, detection.bbox.y,
//                           detection.bbox.width, detection.bbox.height);
//             cv::rectangle(output_image, bbox, cv::Scalar(0, 255, 0), 2);

//             // Draw keypoints
//             for (const auto &keypoint : detection.keypoints.keypoints_2) {
//                 cv::Point2f pt(keypoint.x, keypoint.y);
//                 cv::circle(output_image, pt, 3, cv::Scalar(255, 0, 0), -1);
//             }
//         }

//         //! Convert to ROS message using cv_bridge
//         std_msgs::msg::Header header;
//         header.stamp = rclcpp::Clock().now();
//         cv_bridge::CvImage cv_bridge_img(header, DefaultPublishEncoding, output_image);
//         msg = *cv_bridge_img.toImageMsg();
//         return 0;
//     }

//     // auxiliary data for easy extension without inheritance
//     std::any auxiliary_data;
//     boost::uuids::uuid uid;
//     std::vector<redoxi_public_msgs::msg::Detection> detections;

//     // the image is for publish only
//     std::optional<cv::Mat> image;
// };

//! Delivery target data type for detection request output port
using DeliveryTargetDataBase = output_port_types::DefaultTargetData<DetectionRequestActionType,
                                                                    DetectionRequestActionDataTrait,
                                                                    DeliverySourceData::PublishMessageType_t>;

class DeliveryTargetData : public DeliveryTargetDataBase
{
  public:
    DeliveryTargetData()
    {
        static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>,
                      "DeliveryTargetData must satisfy DeliveryTargetDataConcept");
    }

    DeliveryTargetData(const Goal_t &goal)
        : DeliveryTargetDataBase(goal)
    {
    }

    virtual int to_publish_message(PublishMessageType_t &msg) const
    {
        auto canvas = image.clone();
        image_utils::draw_detections(&canvas, detections);
        if (canvas.empty()) {
            // cannot convert, return error
            return -1;
        }

        //! Convert drawn image to ROS message using cv_bridge
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();
        cv_bridge::CvImage cv_bridge_img(header, DeliverySourceData::DefaultEncoding, canvas);
        msg = *cv_bridge_img.toImageMsg();
        return 0;
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

    // the original image, used for visualization
    cv::Mat image;
    std::vector<redoxi_public_msgs::msg::Detection> detections;
};

//! Stamp data type for detection request output port
using DeliveryStampData = output_port_types::DefaultStampData;

//! Delivery policy type for detection request output port
using DeliveryPolicy = output_port_types::DefaultDeliveryPolicy<RetryPolicy>;
static_assert(output_port_types::DeliveryPolicyConcept<DeliveryPolicy>,
              "DeliveryPolicy must satisfy DeliveryPolicyConcept");

//! Request type for detection request output port
using DeliveryRequestBase = output_port_types::DefaultDeliveryRequest<DeliverySourceData,
                                                                      DeliveryTargetData,
                                                                      DeliveryPolicy,
                                                                      DeliveryStampData>;

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
        const auto &source_data = this->m_source_data;
        goal.detections = source_data.detections;
        if (source_data.image.has_value()) {
            target_data.image = source_data.image.value();
            std_msgs::msg::Header header;
            header.stamp = rclcpp::Clock().now();
            goal.frame.raw_image = *cv_bridge::CvImage(header,
                                                       DeliverySourceData::DefaultPublishEncoding,
                                                       target_data.image)
                                        .toImageMsg();
            goal.frame.metadata.width = target_data.image.cols;
            goal.frame.metadata.height = target_data.image.rows;
        }

        // set additional information into the goal
        using ActionTrait = DeliveryTargetData::ActionDataTrait_t;

        // set the source data UUID
        ActionTrait::set_uuid(goal, source_data.get_uuid());

        // set the control signal code
        ActionTrait::mark_with_control_signal(goal, get_control_signal_code());

        return 0;
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

    // custom function to transform the request to target data
    std::function<void(DeliveryTargetData &, const DeliveryRequest &)> custom_to_target_data;
};

static_assert(output_port_types::DeliveryRequestConcept<DeliveryRequest>,
              "DeliveryRequest must satisfy DeliveryRequestConcept");

using DeliveryTask = output_port_types::DefaultDeliveryTask<DeliveryRequest,
                                                            DeliveryTargetData,
                                                            RetryPolicy>;
static_assert(output_port_types::DeliveryTaskConcept<DeliveryTask>,
              "DeliveryTask must satisfy DeliveryTaskConcept");

//! Downstream debug publisher type for detection request output port
using DownstreamDebugPublisher = redoxi_works::image_ports::types::DownstreamDebugPublisher;

//! Downstream spec type for detection request output port
using DownstreamSpec = output_port_types::DefaultDownstreamSpec<DetectionRequestActionType,
                                                                DeliveryPolicy,
                                                                DownstreamDebugPublisher,
                                                                DownstreamDebugPublisher>;
static_assert(output_port_types::DownstreamSpecConcept<DownstreamSpec>,
              "DownstreamSpec must satisfy DownstreamSpecConcept");

//! Init config type for detection request output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec>;

//! Downstream type for detection request output port
using Downstream = output_port_types::DefaultDownstream<DownstreamSpec>;

//! Detection request output port spec
struct DetectionRequestOutputPortSpec {
    DetectionRequestOutputPortSpec()
    {
        static_assert(output_port_types::AsyncActionOutputPortSpecConcept<DetectionRequestOutputPortSpec>,
                      "DetectionRequestOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
    }

    //! Action type and related types
    using ActionType_t = DetectionRequestActionType;
    using ActionGoal_t = ActionType_t::Goal;
    using ActionResult_t = ActionType_t::Result;
    using ActionFeedback_t = ActionType_t::Feedback;
    using ActionDataTrait_t = DetectionRequestActionDataTrait;

    //! Time unit type
    using TimeUnit_t = TimeUnit;

    //! Retry policy type
    using RetryPolicy_t = RetryPolicy;

    //! Source data type
    using DeliverySourceData_t = DeliverySourceData;

    //! Source data publish message type
    using SourcePublishMessageType_t = DeliverySourceData::PublishMessageType_t;

    //! Source data publisher type
    using SourcePublisherType_t = DownstreamDebugPublisher;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data publish message type
    using TargetPublishMessageType_t = DeliveryTargetData::PublishMessageType_t;

    //! Target data publisher type
    using TargetPublisherType_t = DownstreamDebugPublisher;

    //! Stamp type
    using DeliveryStamp_t = DeliveryStampData;

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


} // namespace redoxi_works::detection_ports::request_response::types