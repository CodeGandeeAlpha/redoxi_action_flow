#include "test_img/test_img.hpp"

using std::placeholders::_1;

namespace FlowRos2Pipeline{
    TestImg::TestImg(): rclcpp::Node("test_img"),
                        k_(0),
                        average_round_time_(0) {
        auto logger = rclcpp::get_logger("test_img");

        // 声明参数
        this->declare_parameter<std::string>("frame_pub", "");

        // 获取参数
        std::string frame_pub_topic = this->get_parameter("frame_pub").as_string();
        RCLCPP_INFO(logger, "[TestImg] frame_pub_topic: %s", frame_pub_topic);

        // 创建publisher
        img_subscripter_ = this->create_subscription<my_msgs::msg::Image1080p>(frame_pub_topic, 1,
                        std::bind(&TestImg::img_send_sub_callback, this, _1));

        RCLCPP_INFO(logger, "[TestImg] init success!");
    }

    void TestImg::img_send_sub_callback(std::shared_ptr<my_msgs::msg::Image1080p> msg) {
        auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();

        auto logger = rclcpp::get_logger("test_img");
        RCLCPP_INFO(
            logger,
            "Received message %zu on address %p", msg->step, msg.get());
        auto msg_timestamp = msg.get()->timestamp;

        auto diff = now - msg_timestamp;

        ++k_;
        average_round_time_ = ((k_ - 1) * average_round_time_ + diff) / k_;

        cv::Mat frame(1080, 1920, CV_8UC3, msg.get()->data.data());

        // cv::imwrite("/mnt/chengxiao/code/psf_ros2_ws/test_outputs/" + std::to_string(msg->step) + ".jpg", frame);
    }

    TestImg::~TestImg()
    {
        auto logger = rclcpp::get_logger("test_img");
        RCLCPP_INFO(logger, "Received %d messages", k_);
        RCLCPP_INFO(
        logger, "Average round time %f milliseconds", static_cast<float>(average_round_time_) / 1e6);
    }
}