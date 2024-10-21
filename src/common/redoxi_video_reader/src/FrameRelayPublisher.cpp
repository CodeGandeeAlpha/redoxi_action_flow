#include <redoxi_video_reader/sinks/FrameRelayPublisher.hpp>

namespace redoxi_works
{
FrameRelayPublisher::FrameRelayPublisher(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
}

void FrameRelayPublisher::init()
{
    // create the action server
    m_frame_receive_action_server = rclcpp_action::create_server<FrameReceiveAction_t>(
        this,
        m_frame_receive_action_name,
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FrameReceiveAction_t::Goal>) {
            //! Handle goal request
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<FrameReceiveGoalHandle_t>) {
            //! Handle cancel request
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<FrameReceiveGoalHandle_t> goal_handle) {
            //! Handle accepted goal
            this->execute_goal(goal_handle);
        });
}
} // namespace redoxi_works
