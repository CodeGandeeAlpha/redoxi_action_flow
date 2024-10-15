#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <redoxi_common_cpp/redoxi_common_cpp.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>

#include <redoxi_video_reader_base/visibility_control.h>


namespace redoxi_works
{
class RedoxiVideoReaderBase;


//! The downstream node for RedoxiVideoReaderBase
class REDOXI_VIDEO_READER_BASE_PUBLIC RedoxiVideoReaderDownstreamNode
{
  public:
    virtual ~RedoxiVideoReaderDownstreamNode() = default;
    RedoxiVideoReaderDownstreamNode()
    {
        retry_strategy = std::make_shared<DefaultRetryStrategy>();
    }

    //! The action to accept the frame for this downstream node
    std::string accept_frame_action;

    //! The retry strategy for this downstream node
    std::shared_ptr<IRetryStrategy> retry_strategy;
};


//! The init config for RedoxiVideoReaderBase or its subclass
class REDOXI_VIDEO_READER_BASE_PUBLIC RedoxiVideoReaderInitConfig
{
  public:
    virtual ~RedoxiVideoReaderInitConfig() = default;

    //! The downstream nodes, indexed by node name
    std::map<std::string, std::shared_ptr<RedoxiVideoReaderDownstreamNode>> downstreams;

    //! Load parameters from node
    virtual void from_parameters(RedoxiVideoReaderBase *node);
};

//! The runtime config for RedoxiVideoReaderBase or its subclass
class REDOXI_VIDEO_READER_BASE_PUBLIC RedoxiVideoReaderRuntimeConfig
{
  public:
    virtual ~RedoxiVideoReaderRuntimeConfig() = default;

    //! The step interval in ms
    double step_interval_ms = DefaultNodeStepIntervalMs;

    //! The frame interval in ms
    double frame_interval_ms = -1;

    //! The output image size. If input is different, resize to output_image_size.
    //! If output_image_size.width=-1 or output_image_size.height=-1, keep aspect ratio.
    cv::Size output_image_size = cv::Size(-1, -1);

    //! Frame reading mode
    enum class ReadFrameMode {
        ReadAll = 0,    //!< Read all frames
        ReadIfReady = 1 //!< Read frame if downstream is ready
    };
    ReadFrameMode read_frame_mode = ReadFrameMode::ReadAll;

    virtual void from_parameters(RedoxiVideoReaderBase *node);
};

//! The internal types for RedoxiVideoReaderBase
//! in subclass, you can override these types to customize the behavior
namespace RedoxiVideoReaderInternalTypes
{
using ACT_AcceptFrame = redoxi_public_msgs::action::ProcessFrame;
using InitConfig = RedoxiVideoReaderInitConfig;
using RuntimeConfig = RedoxiVideoReaderRuntimeConfig;
using MSG_Frame = redoxi_public_msgs::msg::Frame;
using GoalHandle = rclcpp_action::ClientGoalHandle<ACT_AcceptFrame>::SharedPtr;

class REDOXI_VIDEO_READER_BASE_PUBLIC Downstream
{
  public:
    virtual ~Downstream() = default;

    // client to call query service
    rclcpp_action::Client<ACT_AcceptFrame>::SharedPtr accept_frame;
    rclcpp_action::Client<ACT_AcceptFrame>::SendGoalOptions accept_frame_options;
};
}; // namespace RedoxiVideoReaderInternalTypes


} // namespace redoxi_works
