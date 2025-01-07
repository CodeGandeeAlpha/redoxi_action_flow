#pragma once

#include <any>
#include <boost/uuid/uuid_generators.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <std_msgs/msg/string.hpp>
#include <psg_private_msgs/msg/psg_document.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <psg_detector/StampedStrPub.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_public_msgs/action/process_keypoints_by_dets.hpp>
#include <redoxi_common_nodes/image_ports/ImageOutputPortSpec.hpp>


namespace redoxi_works
{

namespace async_action_get_keypoints_output_port
{
using TimeUnit = DefaultTimeUnit_t;
using DeliveryActionType = redoxi_public_msgs::action::ProcessKeypointsByDets;
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
class DeliverySourceData : public output_port_types::SimpleImageSourceData
{
  public:
    using VisualizationPublisher_t = image_ports::types::DeliverySourceData::VisualizationPublisher_t;
    using MultiDeviceFrame_t = redoxi_public_msgs::msg::MultiDeviceFrame;
    using VecDetection_t = std::vector<redoxi_public_msgs::msg::Detection>;
    inline static constexpr const char *DefaultEncoding = "bgr8";

    DeliverySourceData()
    {
        static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>, "DeliverySourceData must satisfy DeliverySourceDataConcept");
    }
    virtual ~DeliverySourceData() = default;

    //! Get the frame
    virtual const MultiDeviceFrame_t &get_frame_bundle() const
    {
        return m_frame_bundle;
    }

    //! Set the frame
    virtual void set_frame_bundle(const MultiDeviceFrame_t &frame_bundle)
    {
        m_frame_bundle = frame_bundle;
    }

    //! Get the detections
    virtual const VecDetection_t &get_detections() const
    {
        return m_detections;
    }

    //! Set the detections
    virtual void set_detections(const VecDetection_t &detections)
    {
        m_detections = detections;
    }

    //! Get the detections indices
    virtual const std::vector<size_t> &get_detections_indices() const
    {
        return m_detections_indices;
    }

    //! Set the detections indices
    virtual void set_detections_indices(const std::vector<size_t> &detections_indices)
    {
        m_detections_indices = detections_indices;
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

  protected:
    VecDetection_t m_detections;
    MultiDeviceFrame_t m_frame_bundle;
    // 要使用的detections的索引，若为空则使用所有detections
    std::vector<size_t> m_detections_indices;
};


//! Delivery target data type for image output port
using DeliveryTargetDataBase =
    output_port_types::DefaultDeliveryTargetData<DeliveryActionType,
                                                 RedoxiActionDataTrait<DeliveryActionType>,
                                                 sensor_msgs::msg::Image>;
class DeliveryTargetData : public DeliveryTargetDataBase
{
  public:
    using VisualizationPublisher_t = image_ports::types::DeliveryTargetData::VisualizationPublisher_t;
    DeliveryTargetData()
    {
        static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>, "DeliveryTargetData must satisfy DeliveryTargetDataConcept");
    }
    DeliveryTargetData(const Goal_t &goal)
        : DeliveryTargetDataBase(goal)
    {
    }

    int to_publish_visualization(PubVisualizationMsgType_t &msg) const override
    {
        image_utils::FrameMediator fm(&get_goal().frame_bundle.primary_frame);
        fm.to_image_msg(msg);
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
  protected:
    virtual int _to_target_data(DeliveryTargetData &target_data) const override
    {
        // apply custom function if set
        if (custom_to_target_data) {
            custom_to_target_data(target_data, *this);
            return 0;
        }

        auto &goal = target_data.get_goal();

        // fill payload
        goal.frame_bundle = this->m_source_data.get_frame_bundle();

        // 根据索引挑选出detections
        if (this->m_source_data.get_detections_indices().size() > 0) {
            for (auto &idx : this->m_source_data.get_detections_indices()) {
                goal.detections.push_back(this->m_source_data.get_detections()[idx]);
            }
        } else {
            goal.detections = this->m_source_data.get_detections();
        }
        // // set additional information into the goal
        // using ActionTrait = DeliveryTargetData::ActionDataTrait_t;

        // // set the source data UUID
        // ActionTrait::set_uuid(goal, this->m_source_data.get_uuid());

        // ActionTrait::mark_with_control_signal(goal, get_control_signal_code());

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


//! Downstream spec type for image output port
using Downstream = image_ports::types::DownstreamBaseWithImagePub<DeliveryActionType, DeliveryPolicy>;
using DownstreamSpec = Downstream::DownstreamSpec_t;
static_assert(output_port_types::DownstreamSpecConcept<DownstreamSpec>,
              "DownstreamSpec must satisfy DefaultDownstreamSpecConcept");

//! Init config type for image output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec,
                                                        DeliverySourceData,
                                                        DeliveryTargetData>;

//! PSG get detections output port spec
//! This type must satisfy the AsyncActionOutputPortSpecConcept
//! Any async output port can use this spec as a template argument
struct PSGGetKeypointsOutputPortSpec {
    PSGGetKeypointsOutputPortSpec()
    {
        static_assert(output_port_types::AsyncActionOutputPortSpecConcept<PSGGetKeypointsOutputPortSpec>,
                      "PSGGetKeypointsOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
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
    using SourcePubVisualizationMsgType_t = typename DeliverySourceData_t::PubVisualizationMsgType_t;

    //! Source data publisher type
    using SourceVisualizationPublisher_t = DeliverySourceData_t::VisualizationPublisher_t;

    //! Source data publisher type
    using SourceDataPublisher_t = DeliverySourceData_t::DataPublisher_t;

    //! Source data publish message type
    using SourcePubDataMsgType_t = typename DeliverySourceData_t::PubDataMsgType_t;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data publish message type
    using TargetPubVisualizationMsgType_t = typename DeliveryTargetData_t::PubVisualizationMsgType_t;

    //! Target data publisher type
    using TargetVisualizationPublisher_t = DeliveryTargetData_t::VisualizationPublisher_t;

    //! Target data publisher type
    using TargetDataPublisher_t = DeliveryTargetData_t::DataPublisher_t;

    //! Target data publish message type
    using TargetPubDataMsgType_t = typename DeliveryTargetData_t::PubDataMsgType_t;

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

    //! Source data probe message type
    using SourcePubProbeMsgType_t = DeliverySourceData_t::PubProbeMsgType_t;

    //! Source data probe publisher type
    using SourceProbePublisher_t = DeliverySourceData_t::ProbePublisher_t;

    //! Target data probe message type
    using TargetPubProbeMsgType_t = DeliveryTargetData_t::PubProbeMsgType_t;

    //! Target data probe publisher type
    using TargetProbePublisher_t = DeliveryTargetData_t::ProbePublisher_t;
};

} // namespace async_action_get_keypoints_output_port

} // namespace redoxi_works