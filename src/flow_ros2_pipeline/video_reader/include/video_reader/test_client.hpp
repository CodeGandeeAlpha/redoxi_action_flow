#include <psg_actions/action/process_psg_document.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace FlowRos2Pipeline{

    class TestClient : public rclcpp::Node {
    public:
        TestClient();
        virtual ~TestClient() {}

        virtual void _step();

        void goal_response_callback(const rclcpp_action::ClientGoalHandle<psg_actions::action::ProcessPsgDocument>::SharedPtr & goal_handle);

        void feedback_callback(
            rclcpp_action::ClientGoalHandle<psg_actions::action::ProcessPsgDocument>::SharedPtr,
            const std::shared_ptr<const psg_actions::action::ProcessPsgDocument::Feedback> feedback);

        void result_callback(const rclcpp_action::ClientGoalHandle<psg_actions::action::ProcessPsgDocument>::WrappedResult & result);

    protected:
        rclcpp_action::Client<psg_actions::action::ProcessPsgDocument>::SharedPtr m_client;
        rclcpp::TimerBase::SharedPtr m_step_timer;
    };

}