#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "redoxi_public_msgs/msg/frame.hpp"


class TestPublicMsgsNode : public rclcpp::Node
{
  public:
    TestPublicMsgsNode()
        : Node("test_public_msgs_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("test_pose", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "test_pose", 10,
            std::bind(&TestPublicMsgsNode::poseCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TestPublicMsgsNode::timerCallback, this));
    }

  private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pose: position (%.2f, %.2f, %.2f), orientation w: %.2f",
                    msg->position.x, msg->position.y, msg->position.z, msg->orientation.w);
    }

    void timerCallback()
    {
        auto message = geometry_msgs::msg::Pose();
        message.position.x = 1.0;
        message.position.y = 2.0;
        message.position.z = 3.0;
        message.orientation.w = 1.0;
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPublicMsgsNode>());
    rclcpp::shutdown();
    return 0;
}
