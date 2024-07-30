#include <chrono>
#include <sstream>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>


#include <video_reader/test_server.hpp>

using namespace std::placeholders;
static const uint64_t sleep_time_ms = 100;

static std::string get_time_string(std::string header, int code, rclcpp::Clock& c){
    std::ostringstream os;
    os << header <<"["<<code<<"]"<< "\t"<<c.now().seconds() <<"/"<<c.now().nanoseconds();
    return os.str();
}

namespace FlowRos2Pipeline{
    using ACT_Document = psg_actions::action::ProcessPsgDocument;
    TestServer::TestServer() : rclcpp::Node("test_server") {
        m_clock = rclcpp::Clock(RCL_ROS_TIME);
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
        // rclcpp::sleep_for(std::chrono::milliseconds(1000));
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        // return rclcpp_action::GoalResponse::REJECT;
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
        rclcpp::sleep_for(std::chrono::milliseconds(50000000000));
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<psg_actions::action::ProcessPsgDocument::Result>();
        auto cnow = m_clock.now();
        for(int i=0; i<5; i++){
            //wait
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            //create a feedback here
            auto feedback_msg = std::make_shared<ACT_Document::Feedback>() ;
            feedback_msg->feedback_msg = get_time_string("[Feedback]", i, m_clock);
            RCLCPP_INFO(this->get_logger(), "Publishing feedback: %s", feedback_msg->feedback_msg.c_str());
            goal_handle->publish_feedback(feedback_msg);
        }

        result->return_code = 1;
        result->return_msg = get_time_string("[Succeed]", 0, m_clock);
        RCLCPP_INFO(this->get_logger(), "Done with: %s", result->return_msg.c_str());
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