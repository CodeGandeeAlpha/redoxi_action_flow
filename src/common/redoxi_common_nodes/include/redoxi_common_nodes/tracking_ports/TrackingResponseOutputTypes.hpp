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
class DeliverySourceData : public output_port_types::SimpleImageSourceData
{
  public:
    using TrackTarget_t = TrackTargetType;
    using Detection_t = redoxi_public_msgs::msg::Detection;
    using FrameData_t = image_ports::types::FrameWithMetadata;
    using VisualizationPublisher_t = image_ports::types::DeliverySourceData::VisualizationPublisher_t;
    using DataPublisher_t = image_ports::types::DeliverySourceData::DataPublisher_t;

  public:
    virtual ~DeliverySourceData() = default;

    // to publish message
    int to_publish_visualization(PubVisualizationMsgType_t &msg) const override
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
        image_utils::DrawDetectionsOptions opt;
        opt.colorization_mode = image_utils::DrawDetectionsOptions::ColorizationMode::SemanticIdentity;
        auto fm_input = frame_data.to_frame_mediator();
        cv::Mat output_image = fm_input.to_cv_image_copy();
        image_utils::draw_detections(&output_image, detections, opt);

        // draw and publish it
        image_utils::FrameMediator fm_output(output_image, frame_data.get_encoding());
        fm_output.to_image_msg(msg);
        return 0;
    }

    int to_publish_data(PubDataMsgType_t &msg) const override
    {
        return to_publish_visualization(msg);
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
    FrameData_t frame_data;                   // the frame data
    std::vector<TrackTarget_t> track_targets; // the track targets
    std::any auxiliary_data;                  // any auxiliary data
};
static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>,
              "DeliverySourceData must satisfy DeliverySourceDataConcept");

//! Delivery target data type for detection request output port
using DeliveryTargetDataBase = output_port_types::DefaultTargetData<TrackingResponseActionType,
                                                                    TrackingResponseActionDataTrait,
                                                                    DeliverySourceData::PubVisualizationMsgType_t>;

class DeliveryTargetData : public DeliveryTargetDataBase
{
  public:
    using FrameData_t = DeliverySourceData::FrameData_t;
    using TrackTarget_t = DeliverySourceData::TrackTarget_t;
    using VisualizationPublisher_t = DeliverySourceData::VisualizationPublisher_t;

    DeliveryTargetData()
    {
        static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>,
                      "DeliveryTargetData must satisfy DeliveryTargetDataConcept");
    }

    DeliveryTargetData(const Goal_t &goal)
        : DeliveryTargetDataBase(goal)
    {
    }

    int to_publish_visualization(PubVisualizationMsgType_t &msg) const override
    {
        DeliverySourceData tmp;
        tmp.set_primary_frame(frame_data);
        tmp.set_track_targets(m_goal.track_targets);
        // RDX_INFO_DEV(nullptr, __func__, true, "[track_targets={}]", m_goal.track_targets.size());
        tmp.to_publish_visualization(msg);
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
        auto fm = source_data.get_primary_frame().to_frame_mediator();
        fm.to_frame_msg(goal.frame_bundle.primary_frame);
        target_data.frame_data = source_data.get_primary_frame();

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
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec,
                                                        DeliverySourceData,
                                                        DeliveryTargetData>;

//! Detection request output port spec
struct TrackingResponseOutputPortSpec {
    TrackingResponseOutputPortSpec()
    {
        static_assert(output_port_types::AsyncActionOutputPortSpecConcept<TrackingResponseOutputPortSpec>,
                      "TrackingResponseOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
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

    //! Source data visualization publisher and message type
    using SourcePubVisualizationMsgType_t = DeliverySourceData_t::PubVisualizationMsgType_t;
    using SourceVisualizationPublisher_t = DeliverySourceData_t::VisualizationPublisher_t;

    //! Source data data publisher and message type
    using SourcePubDataMsgType_t = DeliverySourceData_t::PubDataMsgType_t;
    using SourceDataPublisher_t = DeliverySourceData_t::DataPublisher_t;

    //! Source data probe message type and publisher type
    using SourcePubProbeMsgType_t = DeliverySourceData_t::PubProbeMsgType_t;
    using SourceProbePublisher_t = DeliverySourceData_t::ProbePublisher_t;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data visualization publisher and message type
    using TargetPubVisualizationMsgType_t = DeliveryTargetData_t::PubVisualizationMsgType_t;
    using TargetVisualizationPublisher_t = DeliveryTargetData_t::VisualizationPublisher_t;

    //! Target data data publisher and message type
    using TargetPubDataMsgType_t = DeliveryTargetData_t::PubDataMsgType_t;
    using TargetDataPublisher_t = DeliveryTargetData_t::DataPublisher_t;

    //! Target data probe message type and publisher type
    using TargetPubProbeMsgType_t = DeliveryTargetData_t::PubProbeMsgType_t;
    using TargetProbePublisher_t = DeliveryTargetData_t::ProbePublisher_t;

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