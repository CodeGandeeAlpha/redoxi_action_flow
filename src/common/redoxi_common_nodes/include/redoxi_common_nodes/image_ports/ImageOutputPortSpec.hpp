#pragma once

#include <any>
#include <variant>
#include <boost/uuid/uuid_generators.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>

#include <redoxi_common_cpp/redoxi_concepts.hpp>
#include <redoxi_common_nodes/async_action_port/AsyncActionOutputTypes.hpp>

#include <redoxi_common_cpp/ros_utils/shm_utils.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>

#include <redoxi_public_msgs/action/process_frame.hpp>
#include <redoxi_public_msgs/msg/frame_metadata.hpp>
// #include <redoxi_public_msgs/msg/process_frame_goal.hpp>
// #include <redoxi_public_msgs/msg/process_frame_data_msg.hpp>

namespace redoxi_works::image_ports::types
{

using TimeUnit = DefaultTimeUnit_t;
using DeliveryActionType = redoxi_public_msgs::action::ProcessFrame;
static_assert(RedoxiActionConcept<DeliveryActionType>, "DeliveryActionType must satisfy RedoxiActionConcept");

struct FrameWithMetadata {
    using Metadata_t = redoxi_public_msgs::msg::FrameMetadata;
    using Frame_t = redoxi_public_msgs::msg::Frame;

    struct RawData {
        cv::Mat image;
        Metadata_t metadata;
    };
    using RawData_t = RawData;

    // cv::Mat image;
    // Metadata_t metadata;
    std::variant<RawData_t, Frame_t> data = RawData_t();

    bool is_empty() const
    {
        bool has_cv_mat = false;
        if (std::holds_alternative<RawData_t>(this->data)) {
            has_cv_mat = !std::get<RawData_t>(this->data).image.empty();
        }

        bool has_frame_msg = false;
        if (std::holds_alternative<Frame_t>(this->data)) {
            const auto &frame_data = std::get<Frame_t>(this->data);
            auto has_raw_image = !frame_data.raw_image.data.empty();
            auto has_shm_data = shm_utils::ShmTokenTraits::is_valid(frame_data.shm_token);
            has_frame_msg = has_raw_image || has_shm_data;
        }
        return !has_cv_mat && !has_frame_msg;
    }

    std::string get_encoding() const
    {
        if (std::holds_alternative<Frame_t>(this->data)) {
            return std::get<Frame_t>(this->data).metadata.encoding;
        } else {
            return std::get<RawData_t>(this->data).metadata.encoding;
        }
    }

    image_utils::FrameMediator to_frame_mediator() const
    {
        if (std::holds_alternative<Frame_t>(this->data)) {
            return image_utils::FrameMediator(&std::get<Frame_t>(this->data));
        } else {
            return image_utils::FrameMediator(std::get<RawData_t>(this->data).image, std::get<RawData_t>(this->data).metadata);
        }
    }

    //! create from frame message, with data copied from the frame message
    int from_frame_msg(const Frame_t &frame)
    {
        this->data = frame;
        return 0;
    }

    int from_raw_data(const RawData_t &raw_data)
    {
        this->data = raw_data;
        return 0;
    }

    template <typename T>
    T &get_data_as()
    {
        return std::get<T>(this->data);
    }

    template <typename T>
    const T &get_data_as() const
    {
        return std::get<T>(this->data);
    }

    Metadata_t &get_metadata()
    {
        if (std::holds_alternative<Frame_t>(this->data)) {
            return std::get<Frame_t>(this->data).metadata;
        } else {
            return std::get<RawData_t>(this->data).metadata;
        }
    }

    const Metadata_t &get_metadata() const
    {
        if (std::holds_alternative<Frame_t>(this->data)) {
            return std::get<Frame_t>(this->data).metadata;
        } else {
            return std::get<RawData_t>(this->data).metadata;
        }
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

class DownstreamDebugPublisher
{
  public:
    using MessageType_t = sensor_msgs::msg::Image;
    using Publisher_t = redoxi_works::StampedImagePub::Publisher_t;
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
        m_pub = std::make_shared<redoxi_works::StampedImagePub>();
        m_pub->init(pub);
        m_header_color = header_color.value_or(DefaultHeaderColor);
        m_header_scale = header_scale.value_or(DefaultHeaderScale);
    }

    //! Get the current publisher of the DownstreamDebugPublisher
    virtual std::shared_ptr<Publisher_t> get_publisher() const
    {
        return m_pub->get_publisher();
    }

    //! Publish an image with the DownstreamDebugPublisher
    virtual int publish(const cv::Mat &image)
    {
        return m_pub->publish(image, DefaultColorImageEncoding.data());
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
    std::shared_ptr<redoxi_works::StampedImagePub> m_pub;
    cv::Scalar m_header_color{DefaultHeaderColor};
    double m_header_scale = DefaultHeaderScale;
};

//! Source data type for image output port
//! This type must satisfy the DeliverySourceDataConcept
class DeliverySourceData : public output_port_types::SimpleImageSourceData
{
  public:
    using FrameData_t = FrameWithMetadata;
    using FrameBundle_t = redoxi_public_msgs::msg::MultiDeviceFrame;

    //! Visualization publisher type for image output port, override the default one in base class
    using VisualizationPublisher_t = DownstreamDebugPublisher;
    using DataPublisher_t = DownstreamDebugPublisher; // also used for data publisher

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
    int to_publish_visualization(PubVisualizationMsgType_t &msg) const override
    {
        // empty primary frame, skip
        if (m_primary_frame.is_empty()) {
            return -1;
        }

        // convert primary frame to ROS message
        auto fm = m_primary_frame.to_frame_mediator();
        fm.to_image_msg(msg);

        auto header = msg.header;
        RDX_INFO_DEV(nullptr, __func__, false,
                     "header frame_id={}, stamp.nanosec={}, stamp.sec={}, data size={}",
                     header.frame_id, header.stamp.nanosec, header.stamp.sec, msg.data.size());

        return 0;
    }

    int to_publish_probe(PubProbeMsgType_t &msg, const std::string &context) const override
    {
        nlohmann::json jsdata = _get_default_probe_json(context);
        jsdata["frame_number"] = m_primary_frame.get_metadata().frame_num;
        msg.data = jsdata.dump();
        return 0;
    }

    int to_publish_data(PubDataMsgType_t &msg) const override
    {
        return to_publish_visualization(msg);
    }

    virtual void from_frame_bundle(const FrameBundle_t &frame_bundle)
    {
        m_primary_frame.from_frame_msg(frame_bundle.primary_frame);

        // for secondary frames
        m_secondary_frames.resize(frame_bundle.secondary_frames.size());
        for (size_t i = 0; i < frame_bundle.secondary_frames.size(); ++i) {
            m_secondary_frames[i].from_frame_msg(frame_bundle.secondary_frames[i]);
        }
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;

  protected:
    FrameWithMetadata m_primary_frame;
    std::vector<FrameWithMetadata> m_secondary_frames;
};
static_assert(output_port_types::DeliverySourceDataConcept<DeliverySourceData>,
              "DeliverySourceData must satisfy DeliverySourceDataConcept");


//! Delivery target data type for image output port
using DeliveryTargetDataBase =
    output_port_types::DefaultDeliveryTargetData<DeliveryActionType,
                                                 RedoxiActionDataTrait<DeliveryActionType>,
                                                 DeliverySourceData::PubVisualizationMsgType_t>;
class DeliveryTargetData : public DeliveryTargetDataBase
{
  public:
    using BaseType_t = DeliveryTargetDataBase;
    using VisualizationPublisher_t = DownstreamDebugPublisher;
    using DataPublisher_t = SimpleRosPublisher<PubDataMsgType_t>;

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
        image_utils::FrameMediator fm(&this->m_goal.frame_bundle.primary_frame);
        auto ret = fm.to_image_msg(msg);
        return ret;
    }

    int to_publish_probe(PubProbeMsgType_t &msg, const std::string &context) const override
    {
        nlohmann::json jsdata = _get_default_probe_json(context);
        jsdata["frame_number"] = image_utils::FrameMediator(&this->m_goal.frame_bundle.primary_frame).get_frame_number();
        msg.data = jsdata.dump();
        return 0;
    }

    // auxiliary data for easy extension without inheritance
    std::any auxiliary_data;
};
static_assert(output_port_types::DeliveryTargetDataConcept<DeliveryTargetData>,
              "DeliveryTargetData must satisfy DeliveryTargetDataConcept");

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
        auto msg_uuid = DeliveryTargetData::ActionDataTrait_t::get_uuid(goal);
        RDX_INFO_DEV(nullptr, __func__, false, "[msg_uuid={}] converting source to target data",
                     UUIDTrait::to_string(msg_uuid));

        // fill primary frame
        {
            const auto &primary_frame = m_source_data.get_primary_frame();
            auto fm = primary_frame.to_frame_mediator();
            fm.to_frame_msg(goal.frame_bundle.primary_frame);
        }

        // fill secondary frames
        {
            const auto &secondary_frames = m_source_data.get_secondary_frames();
            goal.frame_bundle.secondary_frames.resize(secondary_frames.size());
            for (size_t i = 0; i < secondary_frames.size(); ++i) {
                const auto &frame = secondary_frames[i];
                auto fm = frame.to_frame_mediator();
                fm.to_frame_msg(goal.frame_bundle.secondary_frames[i]);
            }
        }

        RDX_INFO_DEV(nullptr, __func__, false, "[msg_uuid={}] Done, converted source to target data",
                     UUIDTrait::to_string(msg_uuid));

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
static_assert(output_port_types::DownstreamSpecConcept<
                  DownstreamSpecWithImagePub<DeliveryActionType, DeliveryPolicy>>,
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

    int init_by_spec(const DownstreamSpec_t &spec, rclcpp::Node *node) override
    {
        return _init_by_spec(spec, node);
    }

    int init_by_spec(const DownstreamSpec_t &spec, rclcpp_lifecycle::LifecycleNode *node) override
    {
        return _init_by_spec(spec, node);
    }

  private:
    template <RosNodeConcept NodeType>
    int _init_by_spec(const DownstreamSpec_t &spec, NodeType *node)
    {
        auto ret = DownstreamBase_t::init_by_spec(spec, node);
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}] failed to initialize downstream", __func__);
        }
        auto qos_source = DownstreamSpec_t::SourceVisualizationPublisher_t::DefaultQoS;
        auto qos_target = DownstreamSpec_t::TargetVisualizationPublisher_t::DefaultQoS;
        using SourceVisMsgType = typename DownstreamSpec_t::SourceVisualizationPublisher_t::MessageType_t;
        using TargetVisMsgType = typename DownstreamSpec_t::TargetVisualizationPublisher_t::MessageType_t;

        {
            auto topic = spec.get_vis_topic_source_data_failed();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_source_data_failed = std::make_shared<DownstreamDebugPublisher>();
                auto pub = node->template create_publisher<SourceVisMsgType>(topic.value(), qos_source);
                this->m_debug_pub_source_data_failed->init(pub);
            }
        }

        {
            auto topic = spec.get_vis_topic_source_data_sending();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_source_data_sending = std::make_shared<DownstreamDebugPublisher>();
                auto pub = node->template create_publisher<SourceVisMsgType>(topic.value(), qos_source);
                this->m_debug_pub_source_data_sending->init(pub);
            }
        }

        {
            auto topic = spec.get_vis_topic_source_data_succeeded();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_source_data_succeeded = std::make_shared<DownstreamDebugPublisher>();
                auto pub = node->template create_publisher<SourceVisMsgType>(topic.value(), qos_source);
                this->m_debug_pub_source_data_succeeded->init(pub);
            }
        }

        {
            auto topic = spec.get_vis_topic_target_data_sending();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_target_data_sending = std::make_shared<DownstreamDebugPublisher>();
                auto pub = node->template create_publisher<TargetVisMsgType>(topic.value(), qos_target);
                this->m_debug_pub_target_data_sending->init(pub);
            }
        }

        {
            auto topic = spec.get_vis_topic_target_data_succeeded();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_target_data_succeeded = std::make_shared<DownstreamDebugPublisher>();
                auto pub = node->template create_publisher<TargetVisMsgType>(topic.value(), qos_target);
                this->m_debug_pub_target_data_succeeded->init(pub);
            }
        }

        {
            auto topic = spec.get_vis_topic_target_data_failed();
            if (topic.has_value() && spec.get_use_debug_publish()) {
                this->m_debug_pub_target_data_failed = std::make_shared<DownstreamDebugPublisher>();
                auto pub = node->template create_publisher<TargetVisMsgType>(topic.value(), qos_target);
                this->m_debug_pub_target_data_failed->init(pub);
            }
        }
        return 0;
    }
};
using Downstream = DownstreamBaseWithImagePub<DeliveryActionType, DeliveryPolicy>;
static_assert(output_port_types::DownstreamConcept<Downstream>,
              "DownstreamSpec must satisfy DownstreamConcept");

//! Init config type for image output port
using InitConfig = output_port_types::DefaultInitConfig<DownstreamSpec,
                                                        DeliverySourceData,
                                                        DeliveryTargetData>;

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
    using SourcePubVisualizationMsgType_t = typename DeliverySourceData_t::PubVisualizationMsgType_t;

    //! Source data publisher type
    using SourceVisualizationPublisher_t = typename DownstreamSpec::SourceVisualizationPublisher_t;

    //! Source data publish message type
    using SourcePubDataMsgType_t = DeliverySourceData_t::PubDataMsgType_t;

    //! Source data publisher type
    using SourceDataPublisher_t = DeliverySourceData_t::DataPublisher_t;

    //! Source data probe message type
    using SourcePubProbeMsgType_t = DeliverySourceData_t::PubProbeMsgType_t;

    //! Source data probe publisher type
    using SourceProbePublisher_t = DeliverySourceData_t::ProbePublisher_t;

    //! Target data type
    using DeliveryTargetData_t = DeliveryTargetData;

    //! Target data publish message type
    using TargetPubVisualizationMsgType_t = typename DeliveryTargetData_t::PubVisualizationMsgType_t;

    //! Target data publisher type
    using TargetVisualizationPublisher_t = typename DownstreamSpec::TargetVisualizationPublisher_t;

    //! Target data publish message type
    using TargetPubDataMsgType_t = typename DeliveryTargetData_t::PubDataMsgType_t;

    //! Target data publisher type
    using TargetDataPublisher_t = DeliveryTargetData_t::DataPublisher_t;

    //! Target data probe message type
    using TargetPubProbeMsgType_t = DeliveryTargetData_t::PubProbeMsgType_t;

    //! Target data probe publisher type
    using TargetProbePublisher_t = DeliveryTargetData_t::ProbePublisher_t;

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