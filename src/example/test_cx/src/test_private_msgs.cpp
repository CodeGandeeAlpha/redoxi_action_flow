#include "geometry_msgs/msg/point.hpp"
#include "psg_private_msgs/msg/person.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

class TestPrivateMsgsNode : public rclcpp::Node
{
  public:
    TestPrivateMsgsNode()
        : Node("test_private_msgs_node")
    {
        publisher_ = this->create_publisher<psg_private_msgs::msg::Person>("test_person", 10);
        subscription_ = this->create_subscription<psg_private_msgs::msg::Person>(
            "test_person", 10,
            std::bind(&TestPrivateMsgsNode::personCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TestPrivateMsgsNode::timerCallback, this));
    }

  private:
    void personCallback(const psg_private_msgs::msg::Person::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received person: track_id %ld, body_height %.2f, body_height_conf %.2f",
                    msg->track_id, msg->body_height, msg->body_height_conf);
    }

    void timerCallback()
    {
        auto message = psg_private_msgs::msg::Person();
        message.uuid.uuid = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
        message.track_id = 1;
        message.body_height = 1.75;
        message.body_height_conf = 0.95;
        message.head_position_3.x = 0.1;
        message.head_position_3.y = 0.2;
        message.head_position_3.z = 1.6;
        message.foot_position_3.x = 0.1;
        message.foot_position_3.y = 0.2;
        message.foot_position_3.z = 0.0;
        publisher_->publish(message);
    }

    rclcpp::Publisher<psg_private_msgs::msg::Person>::SharedPtr publisher_;
    rclcpp::Subscription<psg_private_msgs::msg::Person>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPrivateMsgsNode>());
    rclcpp::shutdown();
    return 0;
}
