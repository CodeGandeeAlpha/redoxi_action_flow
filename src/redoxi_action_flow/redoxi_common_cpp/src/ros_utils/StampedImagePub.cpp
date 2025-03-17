#include <redoxi_common_cpp/_pch.hpp>

#include "redoxi_common_cpp/ros_utils/StampedImagePub.hpp"
#include <redoxi_common_cpp/image_proc/ImageStamper.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace redoxi_works
{

StampedImagePub::StampedImagePub(rclcpp::Node *node, const std::string &topic_name, std::optional<rclcpp::QoS> qos)
    : m_node(node)
{
    init(node, topic_name, qos);
}

StampedImagePub::StampedImagePub(rclcpp_lifecycle::LifecycleNode *node, const std::string &topic_name, std::optional<rclcpp::QoS> qos)
    : m_lifecycle_node(node)
{
    init(node, topic_name, qos);
}

void StampedImagePub::init(std::shared_ptr<Publisher_t> pub)
{
    m_pub = pub;
}

int StampedImagePub::init(rclcpp::Node *node,
                          const std::string &topic_name,
                          std::optional<rclcpp::QoS> qos)
{
    RDX_ASSERT_CHECK_TRUE(node, "Node should not be nullptr in {}", __func__);
    RDX_ASSERT_CHECK_TRUE(!m_pub, "Publisher should not be initialized in {}", __func__);

    m_node = node;
    if (qos) {
        m_pub = node->create_publisher<sensor_msgs::msg::Image>(topic_name, *qos);
    } else {
        m_pub = node->create_publisher<sensor_msgs::msg::Image>(topic_name, DefaultParams::get_debug_publisher_qos());
    }
    return m_pub != nullptr ? 0 : -1;
}

int StampedImagePub::init(rclcpp_lifecycle::LifecycleNode *node,
                          const std::string &topic_name,
                          std::optional<rclcpp::QoS> qos)
{
    RDX_ASSERT_CHECK_TRUE(node, "Node should not be nullptr in {}", __func__);
    RDX_ASSERT_CHECK_TRUE(!m_pub, "Publisher should not be initialized in {}", __func__);

    m_lifecycle_node = node;
    if (qos) {
        m_pub = node->create_publisher<sensor_msgs::msg::Image>(topic_name, *qos);
    } else {
        m_pub = node->create_publisher<sensor_msgs::msg::Image>(topic_name, DefaultParams::get_debug_publisher_qos());
    }
    return m_pub != nullptr ? 0 : -1;
}

StampedImagePub::Publisher_t::SharedPtr StampedImagePub::get_publisher() const
{
    return m_pub;
}

int StampedImagePub::publish(const sensor_msgs::msg::Image &image_msg,
                             std::optional<std::string> text,
                             std::optional<cv::Scalar> text_color,
                             double scale,
                             std::optional<cv::Scalar> background_color)
{
    if (!m_pub) {
        RDX_RAISE_ERROR("Publisher is not initialized in {}", __func__);
        return -1;
    }
    //! Log the image encoding and data size
    RDX_INFO_DEV(nullptr, __func__, "Image encoding: {}, data size: {}", image_msg.encoding, image_msg.data.size());

    //! If image is empty, treat it as error, ignore it
    if (image_msg.data.empty()) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "Image data is empty, treating as error and ignoring.");
        return -1;
    }

    // convert to cvmat
    redoxi_public_msgs::msg::Frame frame;
    frame.raw_image = image_msg;
    image_utils::FrameMediator::make_metadata_compatible(&frame.metadata, image_msg);
    image_utils::FrameMediator fm(&frame);
    cv::Mat cv_image;
    fm.to_cv_image_copy(cv_image);
    // fm.to_cv_image_copy(cv_image, DefaultColorImageEncoding.data());

    // cv::imshow("cv_image", cv_image);
    // cv::waitKey(0);

    // //! Convert sensor_msgs::msg::Image to cv::Mat
    // cv_bridge::CvImageConstPtr cv_ptr;
    // try {
    //     // cv_ptr = cv_bridge::toCvCopy(image_msg, DefaultColorImageEncoding.data());
    //     cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
    //     // cv_ptr = cv_bridge::toCvCopy(image_msg);
    //     RDX_INFO_DEV(nullptr, __func__, "{}", "Successfully converted sensor_msgs::msg::Image to cv::Mat.");
    // } catch (cv_bridge::Exception &e) {
    //     RDX_RAISE_ERROR("cv_bridge exception in {}: {}", __func__, e.what());
    //     return -1;
    // }

    //! Call the publish function that takes cv::Mat
    RDX_INFO_DEV(nullptr, __func__, "{}", "Publishing image using cv::Mat");
    return publish(cv_image, fm.get_encoding(), text, text_color, scale, background_color);
}


int StampedImagePub::publish(const cv::Mat &image,
                             std::string encoding,
                             std::optional<std::string> text,
                             std::optional<cv::Scalar> text_color,
                             double scale,
                             std::optional<cv::Scalar> background_color)
{
    if (!m_pub) {
        RDX_RAISE_ERROR("Publisher is not initialized in {}", __func__);
        return -1;
    }

    //! If image is empty, treat it as error, ignore it
    if (image.empty()) {
        // m_pub->publish(sensor_msgs::msg::Image());
        return -1;
    }

    cv::Mat stamped_image = image;
    if (text) {
        ImageStamper stamper(image);
        stamper.add_text(*text, scale, text_color, background_color);
        stamped_image = stamper.stamp(false);
    }

    image_utils::FrameMediator fm(stamped_image, encoding);
    sensor_msgs::msg::Image msg;
    fm.to_image_msg(msg);

    RDX_INFO_DEV(nullptr, __func__, "StampedImagePub::publish: Publishing image with encoding: {}", msg.encoding);
    // {
    //     // convert to bgr8 and imshow it
    //     // cv::Mat bgr_image;
    //     // cv::cvtColor(stamped_image, bgr_image, cv::COLOR_RGB2BGR);
    //     cv::imshow("stamped_image", stamped_image);
    //     cv::waitKey(0);
    // }
    // sensor_msgs::msg::Image::SharedPtr msg =
    //     cv_bridge::CvImage(std_msgs::msg::Header(),
    //                        DefaultColorImageEncoding.data(),
    //                        stamped_image)
    //         .toImageMsg();
    // msg->header.stamp = m_node->now();
    m_pub->publish(msg);

    return 0;
}


} // namespace redoxi_works