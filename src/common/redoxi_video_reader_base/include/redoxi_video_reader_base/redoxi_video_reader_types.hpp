#pragma once

#include <string>
#include <map>
#include <memory>
#include <optional>

#include <opencv2/opencv.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>
#include <redoxi_video_reader_base/visibility_control.h>


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
class REDOXI_VIDEO_READER_BASE_PUBLIC DownstreamNode
{
  public:
    virtual ~DownstreamNode() = default;
    DownstreamNode()
    {
        retry_strategy = std::make_shared<DefaultRetryStrategy>();
    }

    //! The action to accept the frame for this downstream node
    std::string accept_frame_action;

    //! The retry strategy for this downstream node
    std::shared_ptr<IRetryStrategy> retry_strategy;
};


//! The init config for RedoxiVideoReaderBase or its subclass
class REDOXI_VIDEO_READER_BASE_PUBLIC InitConfig
{
  public:
    virtual ~InitConfig() = default;

    //! The downstream nodes, indexed by node name
    std::map<std::string, std::shared_ptr<DownstreamNode>> downstreams;

    //! Load parameters from node
    virtual void from_parameters(RedoxiVideoReaderBase *)
    {
    }
};

//! The runtime config for RedoxiVideoReaderBase or its subclass
class REDOXI_VIDEO_READER_BASE_PUBLIC RuntimeConfig
{
  public:
    virtual ~RuntimeConfig() = default;

    //! The step interval in ms
    double step_interval_ms = DefaultNodeStepIntervalMs;

    //! The frame interval in ms, 0 means as fast as possible
    double frame_interval_ms = 0;

    //! The output image size. If input is different, resize to output_image_size.
    //! If output_image_size.width=-1 or output_image_size.height=-1, keep aspect ratio.
    cv::Size output_image_size = cv::Size(-1, -1);

    //! Frame reading mode
    enum class ReadFrameMode {
        ReadAll = 0,    //!< Read all frames
        ReadIfReady = 1 //!< Read frame if downstream is ready
    };
    ReadFrameMode read_frame_mode = ReadFrameMode::ReadAll;

    virtual void from_parameters(RedoxiVideoReaderBase *)
    {
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
class REDOXI_VIDEO_READER_BASE_PUBLIC Downstream
{
  public:
    virtual ~Downstream() = default;

    using GoalClient_t = rclcpp_action::Client<InternalTypes::ACT_AcceptFrame>;
    using GoalHandle_t = GoalClient_t::GoalHandle;
    using Goal_t = GoalClient_t::Goal;
    using SendGoalOptions_t = GoalClient_t::SendGoalOptions;

    // client to call query service
    GoalClient_t::SharedPtr accept_frame;
    GoalClient_t::SendGoalOptions accept_frame_options;
};

/**
 * @brief The result of sending a frame to downstream
 * @details If error_code is set, it is already resolved, you can use
 *          goal_handle_future.get() to get the goal handle without waiting,
 *          otherwise, you can use goal_handle_future.wait() to wait for the result
 */
struct REDOXI_VIDEO_READER_BASE_PUBLIC SendFrameResult {
    /**
     * @brief The error code, 0 means success, otherwise means failure
     * @note If it is not set, it means the result is unknown
     */
    std::optional<int> error_code;

    /**
     * @brief The goal handle future, got from ROS action
     * @details You can resolve it by yourself to get the goal handle
     *          If error_code is set to 0, it means the goal is accepted,
     *          you can use goal_handle_future.get() to get the goal handle without waiting.
     *          If error_code is set to non-zero, it means we do not know the result yet,
     *          you can use goal_handle_future.wait() to wait for the result.
     */
    std::shared_future<Downstream::GoalHandle_t::SharedPtr> goal_handle_future;
};

//! The frame delivery task
class REDOXI_VIDEO_READER_BASE_PUBLIC FrameDeliveryTask
{
  public:
    virtual ~FrameDeliveryTask() = default;
    const boost::uuids::uuid &create_uuid()
    {
        // generate a random uuid
        uid = boost::uuids::random_generator()();
        return uid;
    }

    boost::uuids::uuid uid;

    //! The frame to deliver
    cv::Mat frame;
    size_t frame_number = 0;
    double timestamp_sec = 0;

    //! The shared memory id of the frame, in v6d
    // uint64_t shared_memory_id = 0;
    // bool is_shared_memory_allocated = false;
};

}; // namespace RedoxiVideoReaderBaseTypes

} // namespace redoxi_works
