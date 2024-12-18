#pragma once

#include <optional>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <redoxi_shared_memory/SharedMemoryTypes.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>
#include <redoxi_public_msgs/msg/frame_metadata.hpp>
#include <redoxi_common_cpp/redoxi_options.hpp>


namespace redoxi_works::image_utils
{
/**
 * @brief A class to read/write frames in a consistent way.
 * @note You must make sure the data is valid during the lifetime of the object.
 */
class FrameMediator
{
  public:
    using Metadata_t = redoxi_public_msgs::msg::FrameMetadata;
    using FrameMsg_t = redoxi_public_msgs::msg::Frame;
    using ImageMsg_t = sensor_msgs::msg::Image;
    using MsgStorageOptions_t = options::MessageStorageOptions;

    inline constexpr static std::string_view AssumedEncoding_3ch_u8 = "bgr8";
    inline constexpr static std::string_view AssumedEncoding_1ch_u8 = "mono8";
    inline constexpr static std::string_view AssumedEncoding_3ch_u16 = "bgr16";
    inline constexpr static std::string_view AssumedEncoding_1ch_u16 = "mono16";

  public:
    /**
     * @brief Construct from metadata only, assuming the frame is all black.
     * @param frame_metadata The frame metadata.
     * @note Assumes frame_metadata contains valid width and height
     * @note Assumes frame_metadata contains valid encoding that matches OpenCV format
     */
    explicit FrameMediator(const Metadata_t &frame_metadata);

    /**
     * @brief Construct from cv::Mat and metadata.
     * @param frame The frame image.
     * @param frame_metadata The frame metadata.
     * @note Assumes frame is not empty
     * @note Assumes frame dimensions match metadata width/height
     * @note Assumes frame type is compatible with metadata encoding
     */
    FrameMediator(const cv::Mat &frame, const Metadata_t &frame_metadata);

    /**
     * @brief Construct from cv::Mat and encoding.
     * @param frame The frame image.
     * @param encoding The image encoding.
     * @note Assumes frame is not empty
     * @note Assumes encoding is a valid OpenCV format string
     * @note Assumes encoding matches the frame type
     */
    FrameMediator(const cv::Mat &frame, const std::string &encoding);

    /**
     * @brief Construct from cv::Mat with default metadata.
     * @param frame The frame image.
     * @note Assumes frame is not empty
     * @note Assumes frame type is one of the standard OpenCV formats
     */
    explicit FrameMediator(const cv::Mat &frame);

    /**
     * @brief Construct from frame message.
     * @param frame_msg The frame message.
     * @note Assumes frame_msg pointer remains valid during object lifetime
     * @note Assumes if frame_msg contains image data, it has valid encoding
     */
    explicit FrameMediator(const FrameMsg_t *frame_msg);

    /**
     * @brief Construct from image message.
     * @param image_msg The image message.
     * @note Assumes image_msg pointer remains valid during object lifetime
     * @note Assumes if image_msg contains image data, it has valid encoding
     */
    // explicit FrameMediator(const ImageMsg_t *image_msg);

    /**
     * @brief Convert to cv::Mat with optional encoding conversion.
     * @param cv_image Output cv::Mat image.
     * @param encoding Optional target encoding.
     * @return 0 on success.
     * @note Assumes if encoding is provided, it's a valid OpenCV format
     * @note Assumes conversion between source and target encoding is possible
     */
    int to_cv_image_copy(cv::Mat &cv_image, std::optional<std::string> encoding = std::nullopt) const;


    /**
     * @brief Get a copy of the frame image.
     * @param encoding Optional target encoding.
     * @return The copied cv::Mat, empty if failed (or the image is actually empty)
     */
    cv::Mat to_cv_image_copy(std::optional<std::string> encoding = std::nullopt) const;

    /**
     * @brief Get shared cv::Mat view of the frame, the memory is shared with the frame message
     * @param cv_image Output cv::Mat image.
     * @return 0 on success.
     * @note Assumes the frame message is valid and the memory is not freed
     */
    int to_cv_image_shared(cv::Mat &output) const;

    /**
     * @brief Get shared cv::Mat view of the frame.
     * @return The shared cv::Mat view.
     * @note Assumes underlying frame data remains valid
     * @note Assumes no encoding conversion is needed
     */
    cv::Mat to_cv_image_shared() const;

    /**
     * @brief Convert to frame message with optional encoding conversion.
     * @param frame_msg Output frame message.
     * @param encoding Optional target encoding.
     * @param storage_options Optional storage options, if not set, use default options
     * @return 0 on success.
     * @note Assumes if encoding is provided, it's a valid OpenCV format
     * @note Assumes conversion between source and target encoding is possible
     */
    int to_frame_msg(redoxi_public_msgs::msg::Frame &frame_msg,
                     std::optional<std::string> encoding = std::nullopt,
                     std::optional<MsgStorageOptions_t> storage_options = std::nullopt) const;

    /**
     * @brief Convert to image message with optional encoding conversion.
     * @param image_msg Output image message.
     * @param encoding Optional target encoding.
     * @return 0 on success.
     * @note Assumes if encoding is provided, it's a valid OpenCV format
     * @note Assumes conversion between source and target encoding is possible
     */
    int to_image_msg(sensor_msgs::msg::Image &image_msg,
                     std::optional<std::string> encoding = std::nullopt) const;

    /**
     * @brief Get device type from metadata.
     * @return The device type.
     * @note Assumes metadata contains valid device type
     */
    int64_t get_device_type() const;

    /**
     * @brief Get device index from metadata.
     * @return The device index.
     * @note Assumes metadata contains valid device index
     */
    int64_t get_device_index() const;

    /**
     * @brief Get frame width from metadata.
     * @return The frame width.
     * @note Assumes metadata contains valid width
     */
    int64_t get_width() const;

    /**
     * @brief Get frame height from metadata.
     * @return The frame height.
     * @note Assumes metadata contains valid height
     */
    int64_t get_height() const;

    /**
     * @brief Get frame number from metadata.
     * @return The frame number.
     * @note Assumes metadata contains valid frame number
     */
    int64_t get_frame_number() const;

    /**
     * @brief Get frame encoding from metadata.
     * @return The frame encoding.
     * @note Assumes metadata contains valid encoding string
     */
    const std::string &get_encoding() const;

    /**
     * @brief Get source frame index from metadata.
     * @return The source frame index.
     * @note Assumes metadata contains valid source frame index
     */
    int64_t get_source_frame_index() const;

    /**
     * @brief Get source timestamp as microseconds.
     * @return The source timestamp in microseconds.
     * @note Assumes metadata contains valid timestamp
     */
    std::chrono::microseconds get_source_timestamp_flat() const;

    /**
     * @brief Get source timestamp as ROS Time.
     * @return The source timestamp.
     * @note Assumes metadata contains valid ROS timestamp
     */
    builtin_interfaces::msg::Time get_source_timestamp() const;

    /**
     * @brief Get frame metadata.
     * @return The frame metadata.
     * @note Assumes metadata is properly initialized
     */
    const Metadata_t &get_metadata() const;

    /**
     * @brief Get frame image.
     * @return The frame image.
     * @note Assumes frame data is valid and matches metadata
     */
    const cv::Mat &_get_frame() const;

    /**
     * @brief Check if cv::Mat is compatible with metadata.
     * @param mat The cv::Mat to check.
     * @param metadata The metadata to check against.
     * @return True if compatible.
     * @note Assumes metadata contains valid dimensions and encoding
     * @note Assumes mat is not empty
     */
    static bool is_compatible(const cv::Mat &mat, const Metadata_t &metadata);
    /**
     * @brief Check if ROS image message is compatible with metadata.
     * @param image_msg The ROS image message to check.
     * @param metadata The metadata to check against.
     * @return True if compatible.
     * @note Assumes metadata contains valid dimensions and encoding
     * @note Assumes image_msg contains valid image data
     */
    static bool is_compatible(const ImageMsg_t &image_msg, const Metadata_t &metadata);

    /**
     * @brief Check if frame message is compatible with metadata.
     * @param frame_msg The frame message to check.
     * @param metadata The metadata to check against.
     * @return True if compatible.
     * @note Assumes metadata contains valid dimensions and encoding
     * @note Assumes frame_msg contains valid image data and metadata
     */
    static bool is_compatible(const FrameMsg_t &frame_msg, const Metadata_t &metadata);

    /**
     * @brief Update frame metadata from CvImage, so that the metadata is compatible with the image,
     *        it will not modify compatible fields, just override those that are not compatible
     * @param frame_metadata Output frame metadata
     * @param cv_img Input CvImage
     * @note Assumes cv_img contains valid image data
     * @note Assumes cv_img encoding is valid
     */
    static void make_metadata_compatible(Metadata_t *frame_metadata, const cv_bridge::CvImage &cv_img);

    /**
     * @brief Update frame metadata from cv::Mat, so that the metadata is compatible with the image,
     *        it will not modify compatible fields, just override those that are not compatible
     * @param frame_metadata Output frame metadata
     * @param cv_img Input cv::Mat
     * @note Assumes cv_img is not empty
     * @note Assumes cv_img type is a standard OpenCV format
     */
    static void make_metadata_compatible(Metadata_t *frame_metadata, const cv::Mat &cv_img);

    /**
     * @brief Update frame metadata from ROS image message, so that the metadata is compatible with the image,
     *        it will not modify compatible fields, just override those that are not compatible
     * @param frame_metadata Output frame metadata
     * @param image_msg Input ROS image message
     * @note Assumes image_msg is not empty
     * @note Assumes image_msg contains valid encoding
     */
    static void make_metadata_compatible(Metadata_t *frame_metadata, const ImageMsg_t &image_msg);

  private:
    cv::Mat m_frame;
    Metadata_t m_frame_metadata;
    const FrameMsg_t *m_frame_msg = nullptr;
};

} // namespace redoxi_works::image_utils
