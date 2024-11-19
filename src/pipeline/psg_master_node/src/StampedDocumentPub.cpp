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

// int StampedDocumentPub::publish(const sensor_msgs::msg::Image &image_msg,
//                                 std::optional<std::string> text,
//                                 std::optional<cv::Scalar> text_color,
//                                 double scale,
//                                 std::optional<cv::Scalar> background_color)
// {
//     //! Convert sensor_msgs::msg::Image to cv::Mat
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try {
//         cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
//     } catch (cv_bridge::Exception &e) {
//         RDX_RAISE_ERROR("cv_bridge exception in {}: {}", __func__, e.what());
//         return -1;
//     }

//     //! Call the publish function that takes cv::Mat
//     return publish(cv_ptr->image, text, text_color, scale, background_color);
// }

// int StampedDocumentPub::publish(const cv::Mat &image,
//                                 std::optional<std::string> text,
//                                 std::optional<cv::Scalar> text_color,
//                                 double scale,
//                                 std::optional<cv::Scalar> background_color)
// {
//     if (!m_pub) {
//         RDX_RAISE_ERROR("Publisher is not initialized in {}", __func__);
//         return -1;
//     }

//     cv::Mat stamped_image = image;
//     if (text) {
//         ImageStamper stamper(image);
//         stamper.add_text(*text, scale, text_color, background_color);
//         stamped_image = stamper.stamp(false);
//     }

//     sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", stamped_image).toImageMsg();
//     msg->header.stamp = m_node->now();
//     m_pub->publish(*msg);

//     return 0;
// }

} // namespace redoxi_works