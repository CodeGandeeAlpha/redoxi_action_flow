#include "psg_detector/StampedFramePub.hpp"
#include <redoxi_common_cpp/image_proc/ImageStamper.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace redoxi_works
{

StampedFramePub::StampedFramePub(rclcpp::Node *node, const std::string &topic_name, std::optional<rclcpp::QoS> qos)
    : m_node(node)
{
    init(node, topic_name, qos);
}

int StampedFramePub::init(rclcpp::Node *node,
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

StampedFramePub::Publisher_t::SharedPtr StampedFramePub::get_publisher() const
{
    return m_pub;
}

int StampedFramePub::publish(const MessageType_t &msg,
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