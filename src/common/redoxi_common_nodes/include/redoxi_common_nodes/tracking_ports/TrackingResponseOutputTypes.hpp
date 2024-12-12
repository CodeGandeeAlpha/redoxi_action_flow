#pragma once

#include <any>
#include <variant>
// #include <ranges>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_nodes/image_ports/ImageOutputPortSpec.hpp>
// #include <redoxi_common_nodes/detection_ports/DetectionResponseCommon.hpp>
#include <redoxi_common_nodes/tracking_ports/TrackingResponseCommon.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
// #include <redoxi_common_nodes/detection_ports/DetectionResponseOutputTypes.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>

namespace redoxi_works::tracking_ports::response_only::types
{

//! Retry policy type implementing the RetryPolicyConcept
using RetryPolicy = redoxi_works::output_port_types::DefaultRetryPolicy<TimeUnit>;

//! Source data type for tracking response output port
//! It encapsulates the frame image and detections which will be sent to tracking nodes to perform tracking
class DeliverySourceData
{
  public:
    using PublishMessageType_t = sensor_msgs::msg::Image;
    using TrackTarget_t = TrackTargetType;
    using Detection_t = redoxi_public_msgs::msg::Detection;
    using FrameData_t = image_ports::types::FrameWithMetadata;

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
        if (frame_data.is_empty()) {
            return -1;
        }

        // draw predicted detections on the frame
        std::vector<Detection_t> detections;
        for (const auto &target : track_targets) {
            detections.push_back(target.predicted_detection);
        }

        // get cv mat from frame data, draw detections on it, and convert to ros message
        image_utils::FrameMediator fm_input(frame_data.image, frame_data.get_encoding());
        cv::Mat output_image = fm_input.to_cv_image_shared().clone();
        image_utils::draw_detections(&output_image, detections);

        // draw and publish it
        image_utils::FrameMediator fm_output(output_image, frame_data.get_encoding());
        fm_output.to_image_msg(msg);
        return 0;
    }


    //! Get the frame image
    virtual const FrameData_t &get_primary_frame() const
    {
        return frame_data;
    }

    //! Get the frame image, mutable version
    virtual FrameData_t &get_primary_frame()
    {
        return frame_data;
    }

    //! Set the frame image
    virtual void set_primary_frame(const FrameData_t &frame_data)
    {
        this->frame_data = frame_data;
    }

    //! Get the track targets
    virtual const std::vector<TrackTarget_t> &get_track_targets() const
    {
        return track_targets;
    }

    //! Get the track targets, mutable version
    virtual std::vector<TrackTarget_t> &get_track_targets()
    {
        return track_targets;
    }

    //! Set the track targets
    virtual void set_track_targets(const std::vector<TrackTarget_t> &track_targets)
    {
        this->track_targets = track_targets;
    }

  protected:
    UUIDType uid;
    FrameData_t frame_data;                   // the frame data
    std::vector<TrackTarget_t> track_targets; // the track targets
    std::any auxiliary_data;                  // any auxiliary data
};
static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>,
              "DeliverySourceData must satisfy DeliverySourceDataConcept");

//! Delivery target data type for detection request output port
using DeliveryTargetDataBase = output_port_types::DefaultTargetData<TrackingResponseActionType,
                                                                    TrackingResponseActionDataTrait,
                                                                    DeliverySourceData::PublishMessageType_t>;

class DeliveryTargetData : public DeliveryTargetDataBase
{
  public:
    using FrameData_t = DeliverySourceData::FrameData_t;
    using TrackTarget_t = DeliverySourceData::TrackTarget_t;

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
        DeliverySourceData tmp;
        tmp.set_primary_frame(frame_data);
        tmp.set_track_targets(m_goal.track_targets);
        tmp.to_publish_message(msg);
        return 0;
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

    // the frame image used for visualization, if not set, will use the raw image in the goal
    FrameData_t frame_data;
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

        goal.track_targets = source_data.get_track_targets();
        source_data.get_primary_frame().to_frame_msg(goal.frame_bundle.primary_frame);
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
    TrackingResponseActionType, DeliveryPolicy>;
using DownstreamSpec = Downstream::DownstreamSpec_t;

//! Init config type for detection request output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec>;

//! Detection request output port spec
struct TrackingRequestOutputPortSpec {
    TrackingRequestOutputPortSpec()
    {
        static_assert(output_port_types::AsyncActionOutputPortSpecConcept<TrackingRequestOutputPortSpec>,
                      "TrackingRequestOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
    }

    //! Action type and related types
    using ActionType_t = TrackingResponseActionType;
    using ActionGoal_t = ActionType_t::Goal;
    using ActionResult_t = ActionType_t::Result;
    using ActionFeedback_t = ActionType_t::Feedback;
    using ActionDataTrait_t = TrackingResponseActionDataTrait;

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

} // namespace redoxi_works::tracking_ports::response_only::types