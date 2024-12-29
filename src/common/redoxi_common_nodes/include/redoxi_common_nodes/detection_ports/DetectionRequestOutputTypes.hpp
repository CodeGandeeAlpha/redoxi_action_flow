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
                                                                    DeliverySourceData::PubVisualizationMsgType_t>;

class DeliveryTargetData : public DeliveryTargetDataBase
{
  public:
    using FrameData_t = image_ports::types::FrameWithMetadata;
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
        auto vis_image = get_visualization_frame();
        if (vis_image.is_empty()) {
            return -1;
        }

        auto fm = vis_image.to_frame_mediator();
        auto canvas = fm.to_cv_image_copy();
        image_utils::draw_detections(&canvas, detections);
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Clock().now();
        cv_bridge::CvImage cv_bridge_img(header, vis_image.get_encoding(), canvas);
        cv_bridge_img.toImageMsg(msg);
        return 0;
    }

    //! Get the image to be visualized
    virtual FrameData_t get_visualization_frame() const
    {
        if (!m_visualization_frame.is_empty()) {
            return m_visualization_frame;
        } else {
            FrameData_t output;
            output.from_frame_msg(m_goal.frame_bundle.primary_frame);
            return output;
        }
    }

    //! Get the image to be visualized, mutable
    FrameData_t &get_visualization_frame()
    {
        return m_visualization_frame;
    }

    //! Set the image to be visualized
    virtual void set_visualization_frame(const FrameData_t &frame)
    {
        m_visualization_frame = frame;
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

  protected:
    // the original image, used for visualization
    FrameData_t m_visualization_frame;
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

        // set the visualization frame
        target_data.set_visualization_frame(source_data.get_primary_frame());

        // {
        //     const auto &raw_image = target_data.get_visualization_frame().get_data_as<image_ports::types::FrameWithMetadata::Frame_t>().raw_image;
        //     RDX_INFO_DEV(nullptr, __func__, "visualization raw_image size={}, empty={}",
        //                  raw_image.data.size(), target_data.get_visualization_frame().is_empty());
        // }

        // convert primary frame to goal
        {
            const auto &frame_data = source_data.get_primary_frame();
            frame_data.to_frame_mediator().to_frame_msg(goal.frame_bundle.primary_frame);
            // {
            //     const auto &raw_image = goal.frame_bundle.primary_frame.raw_image;
            //     RDX_INFO_DEV(nullptr, __func__, "primary_frame raw_image size={}", raw_image.data.size());
            // }
        }

        // convert secondary frame to goal
        {
            auto &secondary_frame = source_data.get_secondary_frames();
            goal.frame_bundle.secondary_frames.clear();
            for (auto &frame : secondary_frame) {
                frame.to_frame_mediator().to_frame_msg(goal.frame_bundle.secondary_frames.emplace_back());
            }
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

using Downstream = image_ports::types::DownstreamBaseWithImagePub<DetectionRequestActionType, DeliveryPolicy>;

using DownstreamSpec = typename Downstream::DownstreamSpec_t;

//! Init config type for detection request output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec,
                                                        DeliverySourceData,
                                                        DeliveryTargetData>;

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
    using SourcePubVisualizationMsgType_t = typename DeliverySourceData::PubVisualizationMsgType_t;

    //! Source data publisher type
    using SourceVisualizationPublisher_t = typename DownstreamSpec::SourceVisualizationPublisher_t;

    //! Source publish data message type
    using SourcePubDataMsgType_t = typename DeliverySourceData::PubDataMsgType_t;

    //! Source data publisher type
    using SourceDataPublisher_t = typename InitConfig::SourceDataPublisher_t;

    //! Source data probe message type
    using SourcePubProbeMsgType_t = DeliverySourceData_t::PubProbeMsgType_t;

    //! Source data probe publisher type
    using SourceProbePublisher_t = DeliverySourceData_t::ProbePublisher_t;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data publish message type
    using TargetPubVisualizationMsgType_t = typename DeliveryTargetData::PubVisualizationMsgType_t;

    //! Target data publisher type
    using TargetVisualizationPublisher_t = typename DownstreamSpec::TargetVisualizationPublisher_t;

    //! Target publish data message type
    using TargetPubDataMsgType_t = typename DeliveryTargetData::PubDataMsgType_t;

    //! Target data publisher type
    using TargetDataPublisher_t = typename InitConfig::TargetDataPublisher_t;

    //! Target data probe message type
    using TargetPubProbeMsgType_t = DeliveryTargetData_t::PubProbeMsgType_t;

    //! Target data probe publisher type
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


} // namespace redoxi_works::detection_ports::request_response::types