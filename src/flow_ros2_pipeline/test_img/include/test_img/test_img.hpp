#ifndef TEST_IMG_HPP_
#define TEST_IMG_HPP_

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>


#include "my_msgs/msg/image1080p.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"


namespace FlowRos2Pipeline{
    class TestImg : public rclcpp::Node {
        public:
            TestImg();
            ~TestImg();
            void img_send_sub_callback(std::shared_ptr<my_msgs::msg::Image1080p> msg);
        private:
            rclcpp::Subscription<my_msgs::msg::Image1080p>::SharedPtr img_subscripter_;
            int64_t k_;
            int64_t average_round_time_;
    };
}

#endif