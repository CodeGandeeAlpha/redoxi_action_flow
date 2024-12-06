#pragma once

#include <any>
#include <variant>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_nodes/image_ports/ImageOutputPortSpec.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionResponseCommon.hpp>
#include <universal_mot_trackers/tracking_ports/TrackingRequestCommon.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionResponseOutputTypes.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>

namespace redoxi_works::model_nodes::tracking_ports::request_response::types
{

//! Retry policy type implementing the RetryPolicyConcept
using RetryPolicy = redoxi_works::output_port_types::DefaultRetryPolicy<TimeUnit>;

//! Source data type for tracking request output port
//! It encapsulates the frame image and detections which will be sent to tracking nodes to perform tracking
class DeliverySourceData
{
  public:
    using PublishMessageType_t = sensor_msgs::msg::Image;
    using Detection_t = redoxi_public_msgs::msg::Detection;
    using FrameMetadata_t = redoxi_public_msgs::msg::FrameMetadata;

  public:
    virtual ~DeliverySourceData() = default;

    // get/set uuid
    UUIDType get_uuid() const
    {
        return this->uid;
    }

    void set_uuid(const UUIDType &uid)
    {
        this->uid = uid;
    }

    // to publish message
    virtual int to_publish_message(PublishMessageType_t &msg) const
    {
        if (frame_image.empty()) {
            return -1;
        }

        cv::Mat output_image = frame_image.clone();
        image_utils::draw_detections(&output_image, detections);

        //! Convert to ROS message using cv_bridge
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();
        cv_bridge::CvImage cv_bridge_img(header, get_image_encoding(), output_image);
        msg = *cv_bridge_img.toImageMsg();
        return 0;
    }


    //! Get the frame image
    virtual const cv::Mat &get_image() const
    {
        return frame_image;
    }

    //! Get the frame image, mutable version
    virtual cv::Mat &get_image()
    {
        return frame_image;
    }

    //! Set the frame image
    virtual void set_image(const cv::Mat &image, const std::string &encoding = "")
    {
        frame_image = image;
        if (!encoding.empty()) {
            frame_metadata.encoding = encoding;
        } else {
            frame_metadata.encoding = image_utils::get_default_image_encoding(image);
        }
        frame_metadata.width = image.cols;
        frame_metadata.height = image.rows;
    }

    //! Get the image encoding
    virtual const std::string &get_image_encoding() const
    {
        return frame_metadata.encoding;
    }

    //! Get the frame metadata
    virtual const FrameMetadata_t &get_frame_metadata() const
    {
        return frame_metadata;
    }

    //! Get the frame metadata, mutable version
    virtual FrameMetadata_t &get_frame_metadata()
    {
        return frame_metadata;
    }

  protected:
    UUIDType uid;
    cv::Mat frame_image; // the frame data
    FrameMetadata_t frame_metadata;
    std::vector<Detection_t> detections; // the detections
    std::any auxiliary_data;             // any auxiliary data
};
static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>,
              "DeliverySourceData must satisfy DeliverySourceDataConcept");

//! Delivery target data type for detection request output port
using DeliveryTargetDataBase = output_port_types::DefaultTargetData<TrackingRequestActionType,
                                                                    TrackingRequestActionDataTrait,
                                                                    DeliverySourceData::PublishMessageType_t>;
//! TODO: finish this
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
        cv::Mat canvas;
        if (!frame_image.empty()) {
            canvas = frame_image.clone();
        } else {
            const auto &raw_image = m_goal.frame.raw_image;
            if (raw_image.data.empty()) {
                return -1;
            }
            canvas = cv_bridge::toCvCopy(raw_image)->image;
        }
        image_utils::draw_detections(&canvas, m_goal.detections);

        //! Convert drawn image to ROS message using cv_bridge
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();
        cv_bridge::CvImage cv_bridge_img(header, DeliverySourceData::DefaultEncoding, canvas);
        msg = *cv_bridge_img.toImageMsg();
        return 0;
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

    // the frame image used for visualization, if not set, will use the raw image in the goal
    cv::Mat frame_image;
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
  protected:
    virtual int _to_target_data(DeliveryTargetData &target_data) const override
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
        goal.frame.metadata = source_data.get_frame_metadata();

        if (!image_msg.empty()) {
            std_msgs::msg::Header header;
            header.stamp = rclcpp::Clock().now();
            source_data.to_publish_message(goal.frame.raw_image);
            goal.frame.metadata.width = image_msg.cols;
            goal.frame.metadata.height = image_msg.rows;
        }

        // standard properties will be set by the base class

        return 0;
    }

  public:
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

using Downstream = image_ports::types::DownstreamBaseWithImagePub<
    DetectionRequestActionType, DeliveryPolicy>;
// using DownstreamDebugPublisher = Downstream::SourcePublisherType_t;
using DownstreamSpec = Downstream::DownstreamSpec_t;

//! Init config type for detection request output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec>;

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
    using SourcePublisherType_t = DownstreamSpec::SourcePublisherType_t;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data publish message type
    using TargetPublishMessageType_t = DeliveryTargetData::PublishMessageType_t;

    //! Target data publisher type
    using TargetPublisherType_t = DownstreamSpec::TargetPublisherType_t;

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

} // namespace redoxi_works::model_nodes::tracking_ports::request_response::types