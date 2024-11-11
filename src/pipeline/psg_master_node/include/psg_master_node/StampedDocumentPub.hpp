#pragma once

#include <psg_private_msgs/msg/psg_document.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <string>
#include <optional>

namespace redoxi_works
{

//! A class to publish a stamped image
class StampedDocumentPub
{
  public:
    using MessageType_t = psg_private_msgs::msg::PsgDocument;
    using Publisher_t = rclcpp::Publisher<MessageType_t>;
    inline static const rclcpp::QoS DefaultQoS = rclcpp::QoS(10).reliable();
    inline static const rclcpp::QoS DefaultUnreliableQoS = rclcpp::QoS(10).best_effort();

    //! Constructor
    StampedDocumentPub() = default;

    //! Constructor that creates publisher during construction
    StampedDocumentPub(rclcpp::Node *node, const std::string &topic_name, std::optional<rclcpp::QoS> qos = std::nullopt);

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
     * @brief Publish the document with a header text
     * @param document The document to publish
     * @param text The text to add to the document
     * @return 0 on success, -1 on failure
     */
    int publish(const psg_private_msgs::msg::PsgDocument &document,
                std::optional<std::string> text = std::nullopt);

  private:
    Publisher_t::SharedPtr m_pub;
    rclcpp::Node *m_node = nullptr;
};
} // namespace redoxi_works