#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <redoxi_public_msgs/action/process_frame.hpp>

namespace redoxi_works
{

//! A sink that publishes the frames received by action/publish/service to a ROS 2 topic
class FrameRelayPublisher : public rclcpp::Node
{
  public:
    //! name of the frame receive action
    inline static std::string DefaultFrameReceiveActionName = "in_action";
    //! name of the image topic
    inline static std::string DefaultImageTopicName = "out_image";

    //! rename the action type to FrameReceiveAction_t
    using FrameReceiveAction_t = redoxi_public_msgs::action::ProcessFrame;
    using FrameReceiveGoalHandle_t = rclcpp_action::ServerGoalHandle<FrameReceiveAction_t>;

  public:
    FrameRelayPublisher(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~FrameRelayPublisher() = default;

    //! Initializes the node, creating the publishers and action servers
    virtual void init();

    //! Sets the name of the frame receive action
    //! @param name The new name for the frame receive action
    void set_frame_receive_action_name(const std::string &name)
    {
        m_frame_receive_action_name = name;
    }

    //! Gets the current name of the frame receive action
    //! @return The current name of the frame receive action
    std::string get_frame_receive_action_name() const
    {
        return m_frame_receive_action_name;
    }

  protected:
    //! The callback function for the frame received event
    virtual void _on_frame(const cv::Mat &frame, const int64_t frame_number);

    //! The callback function for the goal request
    virtual rclcpp_action::GoalResponse
        _handle_frame_receive_goal(const rclcpp_action::GoalUUID &uuid,
                                   std::shared_ptr<const FrameReceiveAction_t::Goal> goal);

    //! The callback function for the accepted goal
    virtual rclcpp_action::GoalResponse
        _handle_frame_receive_accepted(
            const std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle);

  protected:
    //! The publisher for the image topic
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher;

    //! action server for frame receiving
    rclcpp_action::Server<FrameReceiveAction_t>::SharedPtr m_frame_receive_action_server;

    //! name of the action server
    std::string m_frame_receive_action_name = DefaultFrameReceiveActionName;
};

} // namespace redoxi_works