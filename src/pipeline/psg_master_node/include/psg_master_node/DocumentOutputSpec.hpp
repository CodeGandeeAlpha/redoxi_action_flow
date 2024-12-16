#pragma once

#include <any>
#include <boost/uuid/uuid_generators.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <psg_private_msgs/action/process_psg_document.hpp>
#include <psg_master_node/StampedDocumentPub.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <redoxi_common_nodes/image_ports/ImageOutputPortSpec.hpp>


namespace redoxi_works
{

namespace async_action_document_output_port
{
using TimeUnit = DefaultTimeUnit_t;
using DeliveryActionType = psg_private_msgs::action::ProcessPsgDocument;
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

//! Source data type for document output port
//! This type must satisfy the DeliverySourceDataConcept
class DeliverySourceData : public output_port_types::SimpleImageSourceData
{
  public:
    using PSGDocument_t = psg_private_msgs::msg::PsgDocument;
    using VisualizationPublisher_t = image_ports::types::DeliverySourceData::VisualizationPublisher_t;
    DeliverySourceData()
    {
        static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>, "DeliverySourceData must satisfy DeliverySourceDataConcept");
        m_uuid = boost::uuids::random_generator()();
    }
    virtual ~DeliverySourceData() = default;


    //! Get the image
    virtual const PSGDocument_t &get_document() const
    {
        return m_document;
    }

    //! Set the image
    virtual void set_document(const PSGDocument_t &document)
    {
        m_document = document;
    }

    //! convert to publish message
    int to_publish_visualization(PubVisualizationMsgType_t &msg) const override
    {
        if (auxiliary_data.has_value()) {
            std::string auxiliary_data_str = std::any_cast<std::string>(auxiliary_data);

            if (auxiliary_data_str == "image") {
                image_utils::FrameMediator fm(&this->m_document.frame_bundle.primary_frame);
                fm.to_image_msg(msg);
            } else if (auxiliary_data_str == "detection") {
                // TODO: 画检测框
            } else if (auxiliary_data_str == "pose") {
                // TODO: 画姿态
            } else if (auxiliary_data_str == "person") {
                // TODO: 画人
            } else if (auxiliary_data_str == "track") {
                // TODO: 画跟踪框
            } else if (auxiliary_data_str == "counter") {
                // TODO: 画计数
            } else {
                // RDX_ERROR_DEV(this, __func__, true, "{}", "Unknown auxiliary data type");
                return -1;
            }
        }
        return 0;
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

  protected:
    // boost::uuids::uuid m_uuid;
    PSGDocument_t m_document;
};


//! Delivery target data type for image output port
using DeliveryTargetDataBase =
    output_port_types::DefaultTargetData<DeliveryActionType,
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
        image_utils::FrameMediator fm(&this->m_goal.document.frame_bundle.primary_frame);
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
        auto document = this->m_source_data.get_document();

        // convert frame to document
        goal.document = document;

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

using Downstream = image_ports::types::DownstreamBaseWithImagePub<DeliveryActionType, DeliveryPolicy>;

//! Downstream spec type for image output port
using DownstreamSpec = Downstream::DownstreamSpec_t;
static_assert(output_port_types::DownstreamSpecConcept<DownstreamSpec>,
              "DownstreamSpec must satisfy DefaultDownstreamSpecConcept");

//! Init config type for image output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec,
                                                        DeliverySourceData::DataPublisher_t,
                                                        DeliveryTargetData::DataPublisher_t>;


//! document output port spec
//! This type must satisfy the AsyncActionOutputPortSpecConcept
//! Any async output port can use this spec as a template argument
struct PSGDocumentOutputPortSpec {
    PSGDocumentOutputPortSpec()
    {
        static_assert(output_port_types::AsyncActionOutputPortSpecConcept<PSGDocumentOutputPortSpec>,
                      "PSGDocumentOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
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
    using SourceVisualizationPublisher_t = DeliverySourceData::VisualizationPublisher_t;

    using SourceDataPublisher_t = DeliverySourceData::DataPublisher_t;
    using SourcePubDataMsgType_t = DeliverySourceData::PubDataMsgType_t;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data publish message type
    using TargetPubVisualizationMsgType_t = typename DeliveryTargetData_t::PubVisualizationMsgType_t;

    //! Target data publisher type
    using TargetVisualizationPublisher_t = DeliveryTargetData::VisualizationPublisher_t;

    using TargetDataPublisher_t = DeliveryTargetData::DataPublisher_t;
    using TargetPubDataMsgType_t = DeliveryTargetData::PubDataMsgType_t;

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

} // namespace async_action_document_output_port

} // namespace redoxi_works