#pragma once

#include <string>
#include <map>
#include <memory>
#include <optional>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_common_cpp/interfaces/IDeliveryRetryPolicy.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <redoxi_video_reader/visibility_control.h>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>


namespace redoxi_works
{
class RedoxiVideoReaderBase;

namespace RedoxiVideoReaderBaseTypes
{

// globally accessible parameters in ROS related to this node
namespace RosParams
{

const std::string StepIntervalMs = "step_interval_ms";
const std::string FrameIntervalMs = "frame_interval_ms";
const std::string OutputImageSize = "output_image_size";
const std::string ReadFrameMode = "read_frame_mode";

} // namespace RosParams

//! The downstream node for RedoxiVideoReaderBase
class REDOXI_VIDEO_READER_PUBLIC DownstreamSpec
{
  public:
    virtual ~DownstreamSpec() = default;
    DownstreamSpec()
    {
        retry_strategy = std::make_shared<DefaultDeliveryRetryPolicy>();
    }

    //! The action to accept the frame for this downstream node
    std::string accept_frame_action;

    //! If use debug pub for this downstream node
    bool use_debug_pub = true;

    //! The retry strategy for this downstream node
    std::shared_ptr<IDeliveryRetryPolicy> retry_strategy;
};

class REDOXI_VIDEO_READER_PUBLIC FrameDeliveryOptions
{
  public:
    virtual ~FrameDeliveryOptions() = default;

    //! If the buffer is full, how to drop frames?
    enum class DropFrameStrategy {
        // FIXME: currently NoDrop only makes sure the task will be enqueued, but it may still be dropped if
        // number of per-downstream delivery retry is exhausted
        //! No dropping, just block the frame reading until the buffer has space
        NoDrop = 0,
        //! Drop frames as needed, only drop frames that cannot be delivered to downstream
        DropAsNeeded = 1
    };

    //! The type of the frame payload
    //! Enum class representing different types of frame payload
    enum class FramePayloadType {
        Uncompressed = 0,               //!< Uncompressed frame data
        JpegEncoded = 1,                //!< JPEG encoded frame data, quality is jpeg_quality
        PngEncoded = 2,                 //!< PNG encoded frame data
        UncompressedBySharedMemory = 3, //!< Uncompressed frame data stored in shared memory
        // JpegEncodedBySharedMemory = 4,  //!< JPEG encoded frame data stored in shared memory, quality is jpeg_quality
        // PngEncodedBySharedMemory = 5,   //!< PNG encoded frame data stored in shared memory
    };

    //! The number of frames to buffer waiting for delivery
    int num_buffer_frames = 1;

    //! If the buffer is full, how to drop frames?
    DropFrameStrategy drop_frame_strategy = DropFrameStrategy::NoDrop;

    //! The type of the frame payload
    FramePayloadType frame_payload_type = FramePayloadType::Uncompressed;

    //! The quality of the jpeg encoded frame
    int jpeg_quality = 90;
};


//! The init config for RedoxiVideoReaderBase or its subclass
class REDOXI_VIDEO_READER_PUBLIC InitConfig
{
  public:
    virtual ~InitConfig() = default;

    //! The downstream nodes, indexed by node name
    std::map<std::string, std::shared_ptr<DownstreamSpec>> downstreams;

    //! create the debug publish topic for this video reader?
    bool create_debug_pub = true;
    int debug_pub_queue_size = 10;
    std::string debug_pub_task_enqueue_name = "debug_port/task_enqueue";
    std::string debug_pub_task_drop_name = "debug_port/task_drop";

    //! Load parameters from node, this will override empty existing parameters
    virtual void from_parameters(RedoxiVideoReaderBase *node);
};

//! The runtime config for RedoxiVideoReaderBase or its subclass
class REDOXI_VIDEO_READER_PUBLIC RuntimeConfig
{
  public:
    inline static const std::string DEFAULT_OUTPUT_IMAGE_ENCODING = "bgr8";

    virtual ~RuntimeConfig() = default;
    RuntimeConfig()
    {
        frame_delivery_options = std::make_shared<FrameDeliveryOptions>();

        // resolve the delivery policy, this will be used as the fallback policy
        delivery_policy_fallback = std::make_shared<DefaultDeliveryRetryPolicy>();
        delivery_policy_fallback->resolve_to_definite();
    }

    //! The step interval in ms
    double step_interval_ms = DefaultNodeStepIntervalMs;

    //! The frame interval in ms, 0 means as fast as possible
    double frame_interval_ms = 0;

    //! The output image size. If input is different, resize to output_image_size.
    //! If output_image_size.width=-1 or output_image_size.height=-1, keep aspect ratio.
    cv::Size output_image_size = cv::Size(-1, -1);

    //! The encoding of the output image, see sensor_msgs::image_encodings for more details
    //! accepts rgb8, bgr8, mono8, mono16
    std::string output_image_encoding = DEFAULT_OUTPUT_IMAGE_ENCODING;

    //! publish in debug topic?
    bool publish_to_debug_topic = false;

    //! The frame delivery quality of service
    std::shared_ptr<FrameDeliveryOptions> frame_delivery_options;

    /**
     * @brief The fallback delivery policy with all parameters resolved.
     *
     * This policy is used in the following scenarios:
     * - If a downstream node does not specify a delivery policy
     * - If a downstream node's delivery policy is missing a particular parameter
     * - For actions that require parameters independent of the downstream node
     */
    std::shared_ptr<IDeliveryRetryPolicy> delivery_policy_fallback;

    //! Load parameters from node, this will override empty existing parameters
    virtual void from_parameters(RedoxiVideoReaderBase *node);

    //! Get the frame interval as std::chrono::duration
    template <typename TimeUnit_t = DefaultTimeUnit_t>
    TimeUnit_t get_frame_interval_as_std_chrono() const
    {
        return std::chrono::microseconds(int64_t(frame_interval_ms * 1e3));
    }

    //! Get the step interval as std::chrono::duration
    template <typename TimeUnit_t = DefaultTimeUnit_t>
    TimeUnit_t get_step_interval_as_std_chrono() const
    {
        return std::chrono::microseconds(int64_t(step_interval_ms * 1e3));
    }
};

//! The internal types for RedoxiVideoReaderBase, very specific types inside class
//! in subclass, you can override these types to customize the behavior
namespace InternalTypes
{
using ACT_AcceptFrame = redoxi_public_msgs::action::ProcessFrame;
using MSG_Frame = redoxi_public_msgs::msg::Frame;

} // namespace InternalTypes

//! The downstream node for RedoxiVideoReaderBase
class REDOXI_VIDEO_READER_PUBLIC Downstream
{
  public:
    virtual ~Downstream() = default;

    using ActionType_t = InternalTypes::ACT_AcceptFrame;
    using ActionClient_t = rclcpp_action::Client<ActionType_t>;
    using GoalHandle_t = ActionClient_t::GoalHandle;
    using Goal_t = ActionClient_t::Goal;
    using SendGoalOptions_t = ActionClient_t::SendGoalOptions;

    // client to call query service
    std::shared_ptr<DownstreamSpec> spec;

    // the resolved delivery policy for this downstream node
    // which will have all the parameters resolved (not optional)
    std::shared_ptr<IDeliveryRetryPolicy> delivery_policy;

    ActionClient_t::SharedPtr accept_frame;
    SendGoalOptions_t accept_frame_options;

    // publisher for debug image
    std::shared_ptr<StampedImagePub> debug_pub_sending; //!< The publisher for sending frame
    std::shared_ptr<StampedImagePub> debug_pub_dropped; //!< The publisher for dropped frame
    std::shared_ptr<StampedImagePub> debug_pub_sent;    //!< The publisher for sent frame successfully

    //! Get the debug pub sending name
    //! @return The debug pub sending name, empty string if spec is not set
    virtual std::string get_debug_pub_sending_name(const DownstreamSpec &spec) const
    {
        std::string topic_name = "debug_port/" + spec.accept_frame_action + "/frame_sending";

        //! Remove duplicate consecutive slashes from topic name
        topic_name.erase(std::unique(topic_name.begin(), topic_name.end(),
                                     [](char a, char b) { return a == '/' && b == '/'; }),
                         topic_name.end());

        return topic_name;
    }

    //! Get the debug pub dropped name
    //! @return The debug pub dropped name, empty string if spec is not set
    virtual std::string get_debug_pub_dropped_name(const DownstreamSpec &spec) const
    {
        std::string topic_name = "debug_port/" + spec.accept_frame_action + "/frame_dropped";

        //! Remove duplicate consecutive slashes from topic name
        topic_name.erase(std::unique(topic_name.begin(), topic_name.end(),
                                     [](char a, char b) { return a == '/' && b == '/'; }),
                         topic_name.end());

        return topic_name;
    }

    //! Get the debug pub sent name
    //! @return The debug pub sent name, empty string if spec is not set
    virtual std::string get_debug_pub_sent_name(const DownstreamSpec &spec) const
    {
        std::string topic_name = "debug_port/" + spec.accept_frame_action + "/frame_sent";

        //! Remove duplicate consecutive slashes from topic name
        topic_name.erase(std::unique(topic_name.begin(), topic_name.end(),
                                     [](char a, char b) { return a == '/' && b == '/'; }),
                         topic_name.end());

        return topic_name;
    }
};

//! The frame delivery task
class REDOXI_VIDEO_READER_PUBLIC FrameDeliveryTask
{
  public:
    virtual ~FrameDeliveryTask() = default;
    const boost::uuids::uuid &create_uuid()
    {
        // generate a random uuid
        uid = boost::uuids::random_generator()();
        return uid;
    }

    //! The uuid of the task
    boost::uuids::uuid uid;

    //! The frame to deliver
    cv::Mat frame;
    int64_t frame_number = 0;
    double timestamp_sec = 0;

    //! control signal, if provided, will override the frame control signal
    std::optional<redoxi_public_msgs::msg::Control> control_signal;

    std::string get_uid_as_string() const
    {
        return boost::uuids::to_string(uid);
    }
};

}; // namespace RedoxiVideoReaderBaseTypes

} // namespace redoxi_works
