#include "psg_detector/StampedStrPub.hpp"

namespace redoxi_works
{

StampedStrPub::StampedStrPub(rclcpp::Node *node, const std::string &topic_name, std::optional<rclcpp::QoS> qos)
    : m_node(node)
{
    init(node, topic_name, qos);
}

int StampedStrPub::init(rclcpp::Node *node,
                        const std::string &topic_name,
                        std::optional<rclcpp::QoS> qos)
{
    RDX_ASSERT_CHECK_TRUE(node, "Node should not be nullptr in {}", __func__);
    RDX_ASSERT_CHECK_TRUE(!m_pub, "Publisher should not be initialized in {}", __func__);

    m_node = node;
    if (qos) {
        m_pub = node->create_publisher<MessageType_t>(topic_name, *qos);
    } else {
        m_pub = node->create_publisher<MessageType_t>(topic_name, DefaultQoS);
    }
    return m_pub != nullptr ? 0 : -1;
}

StampedStrPub::Publisher_t::SharedPtr StampedStrPub::get_publisher() const
{
    return m_pub;
}

int StampedStrPub::publish(const MessageType_t &msg,
                           std::optional<std::string> text)
{
    if (!m_pub) {
        RDX_RAISE_ERROR("Publisher is not initialized in {}", __func__);
        return -1;
    }

    m_pub->publish(msg);

    return 0;
}
} // namespace redoxi_works