#pragma once

#include <opencv2/opencv.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <optional>

namespace redoxi_works
{

//! A class to publish a stamped image
class StampedImagePub
{
  public:
    using Publisher_t = rclcpp::Publisher<sensor_msgs::msg::Image>;
    inline static constexpr int DEFAULT_QUEUE_SIZE = 10;

    //! Constructor
    StampedImagePub() = default;

    //! Constructor that creates publisher during construction
    StampedImagePub(rclcpp::Node *node, const std::string &topic_name, int queue_size = DEFAULT_QUEUE_SIZE);

    //! Check if the publisher is valid
    bool valid() const
    {
        return m_pub != nullptr;
    }

    //! Initialize the publisher
    int init(rclcpp::Node *node,
             const std::string &topic_name,
             int queue_size = DEFAULT_QUEUE_SIZE);

    //! Get the publisher
    Publisher_t::SharedPtr get_publisher() const;

    /**
     * @brief Publish the image
     * @param image The image to publish
     * @param text The text to add to the image
     * @param text_color The color of the text
     * @param scale The scale of the text
     * @param background_color The background color of the text
     * @return 0 on success, -1 on failure
     */
    int publish(const cv::Mat &image,
                std::optional<std::string> text = std::nullopt,
                std::optional<cv::Scalar> text_color = std::nullopt,
                double scale = 1.0,
                std::optional<cv::Scalar> background_color = std::nullopt);

    //! Publish the sensor_msgs::msg::Image
    int publish(const sensor_msgs::msg::Image &image_msg,
                std::optional<std::string> text = std::nullopt,
                std::optional<cv::Scalar> text_color = std::nullopt,
                double scale = 1.0,
                std::optional<cv::Scalar> background_color = std::nullopt);

  private:
    Publisher_t::SharedPtr m_pub;
    rclcpp::Node *m_node = nullptr;
};
} // namespace redoxi_works