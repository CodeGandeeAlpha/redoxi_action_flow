#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include <video_reader/test_server.hpp>

using namespace std::placeholders;

namespace FlowRos2Pipeline{

    TestServer::TestServer() : rclcpp::Node("test_server") {
        m_server = rclcpp_action::create_server<psg_actions::action::ProcessPsgDocument>(
                this,
                "process_psg_document",
                std::bind(&TestServer::handle_goal, this, _1, _2),
                std::bind(&TestServer::handle_cancel, this, _1),
                std::bind(&TestServer::handle_accepted, this, _1));
    }

    rclcpp_action::GoalResponse TestServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const psg_actions::action::ProcessPsgDocument::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with frame_id %s", goal->document.header.frame_id.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse TestServer::handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<psg_actions::action::ProcessPsgDocument>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void TestServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<psg_actions::action::ProcessPsgDocument>> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<psg_actions::action::ProcessPsgDocument::Result>();
        result->return_code = 1;
        result->return_msg = "success";
        goal_handle->succeed(result);
    }

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FlowRos2Pipeline::TestServer>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}