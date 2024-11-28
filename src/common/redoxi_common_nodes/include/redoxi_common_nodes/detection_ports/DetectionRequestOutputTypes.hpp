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
        const auto &image_msg = source_data.get_image();
        target_data.image = image_msg;

        if (!image_msg.empty()) {
            std_msgs::msg::Header header;
            header.stamp = rclcpp::Clock().now();
            source_data.to_publish_message(goal.frame.raw_image);
            goal.frame.metadata.width = image_msg.cols;
            goal.frame.metadata.height = image_msg.rows;
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