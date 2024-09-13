#include <future>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/types.hpp>
#include <unistd.h>
#include <video_reader/test_client.hpp>
using namespace std::placeholders;

static std::string get_time_string(std::string header, int code, rclcpp::Clock &c)
{
    std::ostringstream os;
    os << header << "[" << code << "]"
       << "\t" << c.now().seconds() << "/" << c.now().nanoseconds();
    return os.str();
}

namespace FlowRos2Pipeline
{

TestClient::TestClient()
    : rclcpp::Node("test_client")
{
    m_clock = rclcpp::Clock(RCL_ROS_TIME);
    m_client = rclcpp_action::create_client<psg_actions::action::ProcessPsgDocument>(this, "process_psg_document");
    while (!m_client->wait_for_action_server(std::chrono::seconds(1))) {
        // RCLCPP_INFO(this->get_logger(), "Waiting for action server to be up...");
    }

    // auto step_timer = this->create_wall_timer(std::chrono::milliseconds(10),
    //         std::bind(&TestClient::_step, this));
    // m_step_timer = step_timer;

    // call step in another thread
    m_step_thread = std::make_shared<std::thread>([this]() {
            while(rclcpp::ok())
            {
                _step();
                rclcpp::sleep_for(std::chrono::nanoseconds(10000000));
            } });
    // rclcpp::executors::SingleThreadedExecutor exec;
    // exec.add_node(this->shared_from_this());
    // exec.spin();
}

void TestClient::goal_response_callback(const rclcpp_action::ClientGoalHandle<psg_actions::action::ProcessPsgDocument>::SharedPtr &goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        // RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void TestClient::feedback_callback(
    rclcpp_action::ClientGoalHandle<psg_actions::action::ProcessPsgDocument>::SharedPtr,
    const std::shared_ptr<const psg_actions::action::ProcessPsgDocument::Feedback> feedback)
{
    // RCLCPP_INFO(this->get_logger(),"%s", feedback->feedback_msg.c_str());
}

void TestClient::result_callback(const rclcpp_action::ClientGoalHandle<psg_actions::action::ProcessPsgDocument>::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_ERROR(this->get_logger(), "Goal was successed, result.msg: %s, result.code: %ld",
                         result.result->return_msg.c_str(), result.result->return_code);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
}

void TestClient::_step()
{
    auto goal = psg_actions::action::ProcessPsgDocument::Goal();
    goal.document.header.frame_id = get_time_string("[Send]", 0, m_clock);
    auto send_goal_options = rclcpp_action::Client<psg_actions::action::ProcessPsgDocument>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&TestClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&TestClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&TestClient::result_callback, this, _1);

    auto res = m_client->async_send_goal(goal, send_goal_options);
    // RCLCPP_INFO(this->get_logger(), "Waiting for response");
    int max_try = 5;
    int n_try = 0;
    // while(rclcpp::ok() && n_try < max_try){
    // //RCLCPP_INFO(this->get_logger(), "Try %d", n_try);
    // // rclcpp::sleep_for(std::chrono::milliseconds(1000));
    // //RCLCPP_INFO(this->get_logger(), "Waiting for future");
    // {
    //     auto status = res.wait_for(std::chrono::milliseconds(5));
    //     switch(status){
    //         case std::future_status::ready:
    //             //RCLCPP_INFO(this->get_logger(), "Future ready");
    //             break;
    //         case std::future_status::timeout:
    //             //RCLCPP_INFO(this->get_logger(), "Future timeout");
    //             break;
    //         case std::future_status::deferred:
    //             //RCLCPP_INFO(this->get_logger(), "Future deferred");
    //             break;
    //     }
    // }

    // wait until goal_response_callback return a value
    auto g = res.get();
    if (g == nullptr) {
        // RCLCPP_INFO(this->get_logger(), "rejected");
    } else {
        switch (g->get_status()) {
            case rclcpp_action::GoalStatus::STATUS_ABORTED:
                // RCLCPP_INFO(this->get_logger(), "STATUS_ABORTED");
                break;
            case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
                // RCLCPP_INFO(this->get_logger(), "STATUS_ACCEPTED");
                break;
            case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
                // RCLCPP_INFO(this->get_logger(), "STATUS_SUCCEEDED");
                break;
            case rclcpp_action::GoalStatus::STATUS_CANCELED:
                // RCLCPP_INFO(this->get_logger(), "STATUS_CANCELED");
                break;
            case rclcpp_action::GoalStatus::STATUS_UNKNOWN:
                // RCLCPP_INFO(this->get_logger(), "STATUS_UNKNOWN");
                break;
            case rclcpp_action::GoalStatus::STATUS_EXECUTING:
                // RCLCPP_INFO(this->get_logger(), "STATUS_EXECUTING");
                break;
            case rclcpp_action::GoalStatus::STATUS_CANCELING:
                // RCLCPP_INFO(this->get_logger(), "STATUS_CANCELING");
                break;
        }
    }

    // RCLCPP_INFO(this->get_logger(), "Waiting for future again");
    {
        auto status = res.wait_for(std::chrono::milliseconds(50000000000000000));
        switch (status) {
            case std::future_status::ready:
                // RCLCPP_INFO(this->get_logger(), "Future ready");
                break;
            case std::future_status::timeout:
                // RCLCPP_INFO(this->get_logger(), "Future timeout");
                break;
            case std::future_status::deferred:
                // RCLCPP_INFO(this->get_logger(), "Future deferred");
                break;
        }
        //     if(status == std::future_status::ready){
        //         break;
        //     }
        if (status == std::future_status::ready) {
            // RCLCPP_INFO(this->get_logger(), "Waiting 3s to see the status");
            sleep(3);
            switch (g->get_status()) {
                case rclcpp_action::GoalStatus::STATUS_ABORTED:
                    // RCLCPP_INFO(this->get_logger(), "STATUS_ABORTED");
                    break;
                case rclcpp_action::GoalStatus::STATUS_ACCEPTED:
                    // RCLCPP_INFO(this->get_logger(), "STATUS_ACCEPTED");
                    break;
                case rclcpp_action::GoalStatus::STATUS_SUCCEEDED:
                    // RCLCPP_INFO(this->get_logger(), "STATUS_SUCCEEDED");
                    break;
                case rclcpp_action::GoalStatus::STATUS_CANCELED:
                    // RCLCPP_INFO(this->get_logger(), "STATUS_CANCELED");
                    break;
                case rclcpp_action::GoalStatus::STATUS_UNKNOWN:
                    // RCLCPP_INFO(this->get_logger(), "STATUS_UNKNOWN");
                    break;
                case rclcpp_action::GoalStatus::STATUS_EXECUTING:
                    // RCLCPP_INFO(this->get_logger(), "STATUS_EXECUTING");
                    break;
                case rclcpp_action::GoalStatus::STATUS_CANCELING:
                    // RCLCPP_INFO(this->get_logger(), "STATUS_CANCELING");
                    break;
            }
        }
    }

    // n_try++;
    // }

    // //RCLCPP_INFO(this->get_logger(), "Status = %d", status);

    // RCLCPP_INFO(this->get_logger(), "Done one step");
}

} // namespace FlowRos2Pipeline

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FlowRos2Pipeline::TestClient>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}