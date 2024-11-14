#pragma once

#include <opencv2/opencv.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <optional>

namespace redoxi_works
{

//! A class to publish a stamped string
class StampedStrPub
{
  public:
    using MessageType_t = std_msgs::msg::String;
    using Publisher_t = rclcpp::Publisher<MessageType_t>;
    inline static const rclcpp::QoS DefaultQoS = rclcpp::QoS(10).reliable();
    inline static const rclcpp::QoS DefaultUnreliableQoS = rclcpp::QoS(10).best_effort();

    //! Constructor
    StampedStrPub() = default;

    //! Constructor that creates publisher during construction
    StampedStrPub(rclcpp::Node *node, const std::string &topic_name, std::optional<rclcpp::QoS> qos = std::nullopt);

    //! Check if the publisher is valid
    bool valid() const
    {
        return m_pub != nullptr;
    }

    //! Initialize the publisher
    int init(rclcpp::Node *node,
             const std::string &topic_name,
             std::optional<rclcpp::QoS> qos = std::nullopt);

    //! Get the publisher
    Publisher_t::SharedPtr get_publisher() const;

    /**
     * @brief Publish the string with a header text
     * @param msg The string to publish
     * @param text The text to add to the string
     * @return 0 on success, -1 on failure
     */
    int publish(const MessageType_t &msg,
                std::optional<std::string> text = std::nullopt);


  private:
    Publisher_t::SharedPtr m_pub;
    rclcpp::Node *m_node = nullptr;
};
} // namespace redoxi_works