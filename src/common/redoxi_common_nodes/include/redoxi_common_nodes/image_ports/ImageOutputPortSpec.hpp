#pragma once

#include <any>
#include <boost/uuid/uuid_generators.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <redoxi_public_msgs/msg/frame_metadata.hpp>


namespace redoxi_works::image_ports::types
{

using TimeUnit = DefaultTimeUnit_t;
using DeliveryActionType = redoxi_public_msgs::action::ProcessFrame;
static_assert(RedoxiActionConcept<DeliveryActionType>, "DeliveryActionType must satisfy RedoxiActionConcept");

struct FrameWithMetadata {
    using Metadata_t = redoxi_public_msgs::msg::FrameMetadata;
    using Frame_t = redoxi_public_msgs::msg::Frame;

    cv::Mat image;
    Metadata_t metadata;

    bool is_empty() const
    {
        return image.empty();
    }

    const std::string &get_encoding() const
    {
        return metadata.encoding;
    }

    int to_frame_msg(Frame_t &frame) const
    {
        if (is_empty()) {
            frame = Frame_t();
        } else {
            image_utils::FrameMediator fm(image, metadata);
            fm.to_frame_msg(frame);
        }
        return 0;
    }

    //! create from frame message, with data copied from the frame message
    int from_frame_msg_copy(const Frame_t &frame)
    {
        image_utils::FrameMediator fm(&frame);
        fm.to_cv_image_copy(image);
        metadata = fm.get_metadata();
        return 0;
    }

    //! create from frame message, with data shared with the frame message
    //! you must ensure the frame message is not destroyed before the frame data
    int from_frame_msg_shared(const redoxi_public_msgs::msg::Frame &frame)
    {
        image_utils::FrameMediator fm(&frame);
        image = fm.to_cv_image_shared();
        metadata = fm.get_metadata();
        return 0;
    }
};

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
class DeliverySourceData
{
  public:
    using FrameData_t = FrameWithMetadata;
    using PublishMessageType_t = sensor_msgs::msg::Image;
    using FrameBundle_t = redoxi_public_msgs::msg::MultiDeviceFrame;

    DeliverySourceData()
    {
        static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>, "DeliverySourceData must satisfy DeliverySourceDataConcept");
        m_uuid = boost::uuids::random_generator()();
    }
    virtual ~DeliverySourceData() = default;

    //! Get the image
    virtual const FrameData_t &get_primary_frame() const
    {
        return m_primary_frame;
    }

    //! Get the image, mutable version
    virtual FrameData_t &get_primary_frame()
    {
        return m_primary_frame;
    }

    //! Set the image
    virtual void set_primary_frame(const FrameData_t &frame)
    {
        m_primary_frame = frame;
    }

    //! Get the secondary frames
    virtual const std::vector<FrameData_t> &get_secondary_frames() const
    {
        return m_secondary_frames;
    }

    //! Get the secondary frames, mutable version
    virtual std::vector<FrameData_t> &get_secondary_frames()
    {
        return m_secondary_frames;
    }

    //! Set the secondary frames
    virtual void set_secondary_frames(const std::vector<FrameData_t> &frames)
    {
        m_secondary_frames = frames;
    }

    //! Convert the source data to a ROS message for publishing
    virtual int to_publish_message(PublishMessageType_t &msg) const
    {
        // empty primary frame, skip
        if (m_primary_frame.is_empty()) {
            return -1;
        }

        // convert primary frame to ROS message
        image_utils::FrameMediator fm(m_primary_frame.image, m_primary_frame.get_encoding());
        fm.to_image_msg(msg);

        return 0;
    }

    //! Get the UUID associated with this source data
    virtual boost::uuids::uuid get_uuid() const
    {
        return m_uuid;
    }

    //! Set the UUID associated with this source data
    virtual void set_uuid(const boost::uuids::uuid &uuid)
    {
        m_uuid = uuid;
    }

    virtual void from_frame_bundle(const FrameBundle_t &frame_bundle)
    {
        image_utils::FrameMediator fm(&frame_bundle.primary_frame);
        fm.to_cv_image_copy(m_primary_frame.image);
        m_primary_frame.metadata = fm.get_metadata();

        // for secondary frames
        m_secondary_frames.resize(frame_bundle.secondary_frames.size());
        for (size_t i = 0; i < frame_bundle.secondary_frames.size(); ++i) {
            image_utils::FrameMediator fm(&frame_bundle.secondary_frames[i]);
            fm.to_cv_image_copy(m_secondary_frames[i].image);
            m_secondary_frames[i].metadata = fm.get_metadata();
        }
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

  protected:
    UUIDType m_uuid;

    FrameWithMetadata m_primary_frame;
    std::vector<FrameWithMetadata> m_secondary_frames;
};


//! Delivery target data type for image output port
using DeliveryTargetDataBase =
    output_port_types::DefaultTargetData<DeliveryActionType, RedoxiActionDataTrait<DeliveryActionType>, DeliverySourceData::PublishMessageType_t>;
class DeliveryTargetData : public DeliveryTargetDataBase
{
  public:
    DeliveryTargetData()
    {
        static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>, "DeliveryTargetData must satisfy DeliveryTargetDataConcept");
    }
    DeliveryTargetData(const Goal_t &goal)
        : DeliveryTargetDataBase(goal)
    {
    }

    virtual int to_publish_message(PublishMessageType_t &msg) const
    {
        image_utils::FrameMediator fm(&this->m_goal.frame_bundle.primary_frame);
        auto ret = fm.to_image_msg(msg);
        return ret;
    }

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

        // fill primary frame
        {
            const auto &primary_frame = m_source_data.get_primary_frame();
            primary_frame.to_frame_msg(goal.frame_bundle.primary_frame);

            // FIXME: source data encoding is wrong, different from request
            RDX_INFO_DEV(nullptr, __func__, false, "in target data goal, raw image encoding={}, in metadata={}",
                         goal.frame_bundle.primary_frame.raw_image.encoding,
                         goal.frame_bundle.primary_frame.metadata.encoding);
        }

        // fill secondary frames
        {
            const auto &secondary_frames = m_source_data.get_secondary_frames();
            goal.frame_bundle.secondary_frames.resize(secondary_frames.size());
            for (size_t i = 0; i < secondary_frames.size(); ++i) {
                const auto &frame = secondary_frames[i];
                frame.to_frame_msg(goal.frame_bundle.secondary_frames[i]);
            }
        }

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

//! Downstream debug publisher type for image output port
class DownstreamDebugPublisher
{
  public:
    using MessageType_t = sensor_msgs::msg::Image;
    using Publisher_t = redoxi_works::StampedImagePub;
    inline static const cv::Scalar DefaultHeaderColor{255, 0, 0};
    inline static constexpr double DefaultHeaderScale = 1.0;
    inline static const rclcpp::QoS DefaultQoS = DefaultParams::DebugPublisherQoS;

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

//! Downstream spec type with image publisher for image output port
template <RosActionConcept ActionType,
          output_port_types::DeliveryPolicyConcept DeliveryPolicyType>
class DownstreamSpecWithImagePub : public output_port_types::DefaultDownstreamSpec<
                                       ActionType,
                                       DeliveryPolicyType,
                                       DownstreamDebugPublisher,
                                       DownstreamDebugPublisher>
{
};
static_assert(output_port_types::DownstreamSpecConcept<DownstreamSpecWithImagePub<DeliveryActionType, DeliveryPolicy>>,
              "DownstreamSpecWithImagePub must satisfy DownstreamSpecConcept");

//! Downstream spec type with image publisher for image output port
using DownstreamSpec = DownstreamSpecWithImagePub<DeliveryActionType, DeliveryPolicy>;
static_assert(output_port_types::DownstreamSpecConcept<DownstreamSpec>,
              "DownstreamSpec must satisfy DefaultDownstreamSpecConcept");

//! Downstream base type with image publisher for image output port
template <RosActionConcept ActionType,
          output_port_types::DeliveryPolicyConcept DeliveryPolicyType>
class DownstreamBaseWithImagePub : public output_port_types::DefaultDownstream<
                                       DownstreamSpecWithImagePub<ActionType, DeliveryPolicyType>>
{
  public:
    using DownstreamBase_t = output_port_types::DefaultDownstream<DownstreamSpecWithImagePub<ActionType, DeliveryPolicyType>>;
    using typename DownstreamBase_t::DownstreamSpec_t;

    virtual int init_by_spec(const DownstreamSpec_t &spec, rclcpp::Node *node)
    {
        auto ret = DownstreamBase_t::init_by_spec(spec, node);
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}] failed to initialize downstream", __func__);
        }
        auto qos_source = DownstreamSpec_t::SourcePublisherType_t::DefaultQoS;
        auto qos_target = DownstreamSpec_t::TargetPublisherType_t::DefaultQoS;
        using SourceInnerPublisherType = typename DownstreamSpec_t::SourcePublisherType_t::Publisher_t;
        using TargetInnerPublisherType = typename DownstreamSpec_t::TargetPublisherType_t::Publisher_t;

        {
            auto topic = spec.get_debug_topic_source_data_failed();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_source_data_failed = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<SourceInnerPublisherType>(node, topic.value(), qos_source);
                this->m_debug_pub_source_data_failed->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_source_data_sending();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_source_data_sending = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<SourceInnerPublisherType>(node, topic.value(), qos_source);
                this->m_debug_pub_source_data_sending->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_source_data_succeeded();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_source_data_succeeded = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<SourceInnerPublisherType>(node, topic.value(), qos_source);
                this->m_debug_pub_source_data_succeeded->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_target_data_sending();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_target_data_sending = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<TargetInnerPublisherType>(node, topic.value(), qos_target);
                this->m_debug_pub_target_data_sending->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_target_data_succeeded();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_target_data_succeeded = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<TargetInnerPublisherType>(node, topic.value(), qos_target);
                this->m_debug_pub_target_data_succeeded->init(pub);
            }
        }

        {
            auto topic = spec.get_debug_topic_target_data_failed();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_target_data_failed = std::make_shared<DownstreamDebugPublisher>();
                auto pub = std::make_shared<TargetInnerPublisherType>(node, topic.value(), qos_target);
                this->m_debug_pub_target_data_failed->init(pub);
            }
        }

        return 0;
    }
};
using Downstream = DownstreamBaseWithImagePub<DeliveryActionType, DeliveryPolicy>;
static_assert(output_port_types::DownstreamConcept<Downstream>,
              "DownstreamSpec must satisfy DefaultDownstreamSpecConcept");

//! Downstream spec type for image output port
// using DownstreamSpec = output_port_types::DefaultDownstreamSpec<
//     DeliveryActionType,
//     DeliveryPolicy,
//     DownstreamDebugPublisher,
//     DownstreamDebugPublisher>;


//! Init config type for image output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec>;

//! Downstream type for image output port
// using DownstreamBase = output_port_types::DefaultDownstream<DownstreamSpec>;
// class Downstream : public DownstreamBase
// {
//   public:
//     Downstream()
//     {
//         static_assert(output_port_types::DownstreamConcept<Downstream>,
//                       "DownstreamSpec must satisfy DefaultDownstreamSpecConcept");
//     }

//     virtual int init_by_spec(const DownstreamSpec &spec, rclcpp::Node *node)
//     {
//         auto ret = DownstreamBase::init_by_spec(spec, node);
//         if (ret != 0) {
//             RDX_RAISE_ERROR("[{}] failed to initialize downstream", __func__);
//         }
//         auto qos_source = DownstreamSpec::SourcePublisherType_t::DefaultQoS;
//         auto qos_target = DownstreamSpec::TargetPublisherType_t::DefaultQoS;
//         using SourceInnerPublisherType = DownstreamSpec::SourcePublisherType_t::Publisher_t;
//         using TargetInnerPublisherType = DownstreamSpec::TargetPublisherType_t::Publisher_t;

//         {
//             auto topic = spec.get_debug_topic_source_data_failed();
//             if (topic.has_value()) {
//                 m_debug_pub_source_data_failed = std::make_shared<DownstreamDebugPublisher>();
//                 auto pub = std::make_shared<SourceInnerPublisherType>(node, topic.value(), qos_source);
//                 m_debug_pub_source_data_failed->init(pub);
//             }
//         }

//         {
//             auto topic = spec.get_debug_topic_source_data_sending();
//             if (topic.has_value()) {
//                 m_debug_pub_source_data_sending = std::make_shared<DownstreamDebugPublisher>();
//                 auto pub = std::make_shared<SourceInnerPublisherType>(node, topic.value(), qos_source);
//                 m_debug_pub_source_data_sending->init(pub);
//             }
//         }

//         {
//             auto topic = spec.get_debug_topic_source_data_succeeded();
//             if (topic.has_value()) {
//                 m_debug_pub_source_data_succeeded = std::make_shared<DownstreamDebugPublisher>();
//                 auto pub = std::make_shared<SourceInnerPublisherType>(node, topic.value(), qos_source);
//                 m_debug_pub_source_data_succeeded->init(pub);
//             }
//         }

//         {
//             auto topic = spec.get_debug_topic_target_data_sending();
//             if (topic.has_value()) {
//                 m_debug_pub_target_data_sending = std::make_shared<DownstreamDebugPublisher>();
//                 auto pub = std::make_shared<TargetInnerPublisherType>(node, topic.value(), qos_target);
//                 m_debug_pub_target_data_sending->init(pub);
//             }
//         }

//         {
//             auto topic = spec.get_debug_topic_target_data_succeeded();
//             if (topic.has_value()) {
//                 m_debug_pub_target_data_succeeded = std::make_shared<DownstreamDebugPublisher>();
//                 auto pub = std::make_shared<TargetInnerPublisherType>(node, topic.value(), qos_target);
//                 m_debug_pub_target_data_succeeded->init(pub);
//             }
//         }

//         {
//             auto topic = spec.get_debug_topic_target_data_failed();
//             if (topic.has_value()) {
//                 m_debug_pub_target_data_failed = std::make_shared<DownstreamDebugPublisher>();
//                 auto pub = std::make_shared<TargetInnerPublisherType>(node, topic.value(), qos_target);
//                 m_debug_pub_target_data_failed->init(pub);
//             }
//         }

//         return 0;
//     }
// };

//! Image output port spec
//! This type must satisfy the AsyncActionOutputPortSpecConcept
//! Any async output port can use this spec as a template argument
struct ImageActionOutputPortSpec {
    ImageActionOutputPortSpec()
    {
        static_assert(output_port_types::AsyncActionOutputPortSpecConcept<ImageActionOutputPortSpec>,
                      "ImageOutputPortSpec must satisfy AsyncActionOutputPortSpecConcept");
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
    using SourcePublishMessageType_t = typename DeliverySourceData_t::PublishMessageType_t;

    //! Source data publisher type
    using SourcePublisherType_t = DownstreamDebugPublisher;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data publish message type
    using TargetPublishMessageType_t = typename DeliveryTargetData_t::PublishMessageType_t;

    //! Target data publisher type
    using TargetPublisherType_t = DownstreamDebugPublisher;

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

} // namespace redoxi_works::image_ports::types