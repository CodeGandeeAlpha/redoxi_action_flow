#include "psg_master_node/StampedDocumentPub.hpp"
#include <redoxi_common_cpp/image_proc/ImageStamper.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace redoxi_works
{

StampedDocumentPub::StampedDocumentPub(rclcpp::Node *node, const std::string &topic_name, std::optional<rclcpp::QoS> qos)
    : m_node(node)
{
    init(node, topic_name, qos);
}

int StampedDocumentPub::init(rclcpp::Node *node,
                             const std::string &topic_name,
                             std::optional<rclcpp::QoS> qos)
{
    RDX_ASSERT_CHECK_TRUE(node, "Node should not be nullptr in {}", __func__);
    RDX_ASSERT_CHECK_TRUE(!m_pub, "Publisher should not be initialized in {}", __func__);

    m_node = node;
    if (qos) {
        m_pub = node->create_publisher<psg_private_msgs::msg::PsgDocument>(topic_name, *qos);
    } else {
        m_pub = node->create_publisher<psg_private_msgs::msg::PsgDocument>(topic_name, DefaultQoS);
    }
    return m_pub != nullptr ? 0 : -1;
}

StampedDocumentPub::Publisher_t::SharedPtr StampedDocumentPub::get_publisher() const
{
    return m_pub;
}

int StampedDocumentPub::publish(const psg_private_msgs::msg::PsgDocument &document,
                                std::optional<std::string> text)
{
    if (!m_pub) {
        RDX_RAISE_ERROR("Publisher is not initialized in {}", __func__);
        return -1;
    }

    // cv::Mat stamped_image = image;
    // if (text) {
    //     ImageStamper stamper(image);
    //     stamper.add_text(*text, scale, text_color, background_color);
    //     stamped_image = stamper.stamp(false);
    // }
    psg_private_msgs::msg::PsgDocument stamped_document = document;

    stamped_document.header.stamp = m_node->now();
    m_pub->publish(stamped_document);

    return 0;
}


} // namespace redoxi_works