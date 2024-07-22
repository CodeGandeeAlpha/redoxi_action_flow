#include <psg_actions/action/process_psg_document.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace FlowRos2Pipeline{

    class TestServer : public rclcpp::Node {
    public:
        TestServer();
        virtual ~TestServer() {}

        rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const psg_actions::action::ProcessPsgDocument::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<psg_actions::action::ProcessPsgDocument>> goal_handle);

        void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<psg_actions::action::ProcessPsgDocument>> goal_handle);

    protected:
        rclcpp_action::Server<psg_actions::action::ProcessPsgDocument>::SharedPtr m_server;
        rclcpp::Clock m_clock;
    };

}