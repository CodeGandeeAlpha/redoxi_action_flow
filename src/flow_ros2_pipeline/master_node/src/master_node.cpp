#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <master_node/master_node.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv){return 1;}

#ifdef COMPILE_MASTER_NODE
class MasterNode : public rclcpp::Node
{
public:
    MasterNode()
    : Node("master_node")
    {
        auto logger = rclcpp::get_logger("master_node");
        // 声明参数
        this->declare_parameter<std::string>("frame_read_pub", "default_value");

        // 获取参数
        std::string frame_read_pub_topic = this->get_parameter("frame_read_pub").as_string();
        RCLCPP_INFO(logger, "[MasterNode] frame_read_pub_topic: %s", frame_read_pub_topic.c_str());

        frame_read_publisher_ = this->create_publisher<std_msgs::msg::Empty>(frame_read_pub_topic, 10);

        timer_ = this->create_wall_timer(10ms, std::bind(&MasterNode::timer_callback, this));

        RCLCPP_INFO(logger, "[MasterNode] init success!");
    }

    void timer_callback()
    {
        auto logger = rclcpp::get_logger("master_node");
        auto message = std_msgs::msg::Empty();
        RCLCPP_INFO(logger, "[MasterNode] frame_read_publisher_->publish()");
        frame_read_publisher_->publish(message);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr frame_read_publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}
#endif