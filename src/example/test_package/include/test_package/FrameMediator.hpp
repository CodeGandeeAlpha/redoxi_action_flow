#include <optional>
#include <chrono>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <redoxi_common_cpp/image_proc/utils.hpp>
#include <redoxi_public_msgs/msg/multi_device_frame.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace redoxi_works
{
// read/write frames in a consistent way
// you must make sure the data is valid during the lifetime of the object
class FrameMediator
{
  public:
    using Metadata_t = redoxi_public_msgs::msg::FrameMetadata;
    using FrameMsg_t = redoxi_public_msgs::msg::Frame;
    inline constexpr static std::string_view AssumedEncoding_3ch_u8 = "bgr8";
    inline constexpr static std::string_view AssumedEncoding_1ch_u8 = "mono8";
    inline constexpr static std::string_view AssumedEncoding_3ch_u16 = "bgr16";
    inline constexpr static std::string_view AssumedEncoding_1ch_u16 = "mono16";

  public:
    // construct from metadata only, assuming the frame is all black
    FrameMediator(const Metadata_t &frame_metadata)
        : m_frame_metadata(frame_metadata)
    {
    }

    // construct from cv::Mat and metadata
    FrameMediator(const cv::Mat &frame, const Metadata_t &frame_metadata)
        : m_frame(frame), m_frame_metadata(frame_metadata)
    {
    }

    // construct from cv::Mat with default metadata
    FrameMediator(const cv::Mat &frame)
        : m_frame(frame), m_frame_metadata()
    {
        update_frame_metadata(m_frame_metadata, frame);
    }

    // construct from frame message
    // note that the frame message is not owned by this object, you need to make sure it is valid during the lifetime of this object
    FrameMediator(const FrameMsg_t *frame_msg)
        : m_frame_msg(frame_msg)
    {
        m_frame_metadata = m_frame_msg->metadata;
    }


  public:
    // convert the frame to cv::Mat, given frame message
    // optionally converting to the specified encoding
    // return 0 if successful, -1 if failed
    int to_cv_image_copy(cv::Mat &cv_image, std::optional<std::string> encoding = std::nullopt) const
    {
        std::string target_encoding = encoding.value_or(get_encoding());
        std::string source_encoding = get_encoding();

        // do we have frame message?
        if (m_frame_msg) {
            // we have frame message, we need to copy the image data
            const auto &raw_image = m_frame_msg->raw_image;
            if (raw_image.data.empty()) {
                // empty frame, just create an empty cv::Mat
                cv_image = cv::Mat();
            } else {
                // convert the image data to cv::Mat
                auto cv_img = cv_bridge::toCvCopy(raw_image, target_encoding);
                cv_image = cv_img->image;
            }
        } else if (m_frame.empty()) {
            // empty frame, just create an empty cv::Mat
            cv_image = cv::Mat();
        } else {
            // we have frame data, just use it, if encoding matches
            if (source_encoding == target_encoding) {
                RDX_INFO_DEV(nullptr, __func__, "source encoding={}, target encoding={}, no conversion", source_encoding, target_encoding);
                cv_image = m_frame;
            } else {
                RDX_INFO_DEV(nullptr, __func__, "source encoding={}, target encoding={}, convert encoding", source_encoding, target_encoding);
                // we need to convert the encoding
                auto cv_img_ptr = std::make_shared<cv_bridge::CvImage>(std_msgs::msg::Header(), source_encoding, m_frame);
                auto converted_cv_img_ptr = cv_bridge::cvtColor(cv_img_ptr, target_encoding);
                cv_image = converted_cv_img_ptr->image;
            }
        }
        return 0;
    }

    // convert the frame to cv::Mat, given frame message
    // the image data will be shared with the frame message, and the encoding is preserved
    // return 0 if successful, -1 if failed
    cv::Mat to_cv_image_shared() const
    {
        // do we have frame message?
        if (m_frame_msg) {
            // we have frame message, just share the image data
            const auto &raw_image = m_frame_msg->raw_image;

            if (raw_image.data.empty()) {
                // empty frame, just create an empty cv::Mat
                return cv::Mat();
            }

            // the dummy object is required by cv_bridge::toCvShare, but it is not used
            // because caller guarantees the frame message is valid during the lifetime of this object
            auto dummy = std::make_shared<int>();
            auto cv_img = cv_bridge::toCvShare(raw_image, dummy);
            return cv_img->image;
        } else {
            // we have frame data, just use it
            return m_frame;
        }
    }

    // convert the frame to redoxi_public_msgs::msg::Frame, given cv::Mat and metadata
    // optionally converting to the specified encoding
    // return 0 if successful, -1 if failed
    int to_frame_msg(redoxi_public_msgs::msg::Frame &frame_msg,
                     std::optional<std::string> encoding = std::nullopt) const
    {
        auto source_encoding = get_encoding();
        auto target_encoding = encoding.value_or(source_encoding);

        // do we have frame message?
        if (m_frame_msg) {
            const auto &raw_image = m_frame_msg->raw_image;

            // for empty image, just copy it
            if (raw_image.data.empty()) {
                frame_msg = *m_frame_msg;
            } else {
                // we have frame message, convert encoding if necessary
                if (source_encoding == target_encoding) {
                    // no conversion is needed, just copy the frame message
                    frame_msg = *m_frame_msg;
                } else {
                    // we need to convert the encoding
                    auto cv_img_ptr = cv_bridge::toCvCopy(m_frame_msg->raw_image, target_encoding);
                    frame_msg.raw_image = *cv_img_ptr->toImageMsg();
                    update_frame_metadata(frame_msg.metadata, *cv_img_ptr);
                }
            }
        } else {
            // we have frame data, convert it to frame message
            if (source_encoding == target_encoding) {
                // no conversion is needed, just copy the frame data
                cv_bridge::CvImage cv_img(std_msgs::msg::Header(), source_encoding, m_frame);
                frame_msg.raw_image = *cv_img.toImageMsg();
                update_frame_metadata(frame_msg.metadata, cv_img);
            } else {
                // we need to convert the encoding
                auto cv_img_ptr = std::make_shared<cv_bridge::CvImage>(
                    std_msgs::msg::Header(), source_encoding, m_frame);
                auto converted_cv_img_ptr = cv_bridge::cvtColor(cv_img_ptr, target_encoding);
                frame_msg.raw_image = *converted_cv_img_ptr->toImageMsg();
                update_frame_metadata(frame_msg.metadata, *converted_cv_img_ptr);
            }
        }
        return 0;
    }

    // get info from metadata
    int64_t get_device_type() const
    {
        return m_frame_metadata.device_tag.device_type;
    }

    // get device index from metadata
    int64_t get_device_index() const
    {
        return m_frame_metadata.device_tag.device_index;
    }

    // frame info
    int64_t get_width() const
    {
        return m_frame_metadata.width;
    }

    // frame info
    int64_t get_height() const
    {
        return m_frame_metadata.height;
    }

    // frame info
    int64_t get_frame_number() const
    {
        return m_frame_metadata.frame_num;
    }

    // encoding follows ros sensor_msgs/image_encodings
    const std::string &get_encoding() const
    {
        return m_frame_metadata.encoding;
    }

    // frame source info
    int64_t get_source_frame_index() const
    {
        return m_frame_metadata.source_frame_index;
    }

    std::chrono::microseconds get_source_timestamp_flat() const
    {
        auto secs = std::chrono::microseconds(m_frame_metadata.source_timestamp.sec * 1000000);
        auto nsecs = std::chrono::microseconds(m_frame_metadata.source_timestamp.nanosec);
        return secs + nsecs;
    }

    builtin_interfaces::msg::Time get_source_timestamp() const
    {
        return m_frame_metadata.source_timestamp;
    }

    // get the frame metadata
    const Metadata_t &get_metadata() const
    {
        return m_frame_metadata;
    }

    // get the cv frame
    const cv::Mat &get_frame() const
    {
        return m_frame;
    }

  private:
    cv::Mat m_frame;
    Metadata_t m_frame_metadata;
    const FrameMsg_t *m_frame_msg = nullptr;

  private:
    void update_frame_metadata(Metadata_t &frame_metadata, const cv_bridge::CvImage &cv_img) const
    {
        frame_metadata.width = cv_img.image.cols;
        frame_metadata.height = cv_img.image.rows;
        frame_metadata.encoding = cv_img.encoding;
    }

    void update_frame_metadata(Metadata_t &frame_metadata, const cv::Mat &cv_img) const
    {
        frame_metadata.width = cv_img.cols;
        frame_metadata.height = cv_img.rows;

        if (cv_img.channels() == 1) {
            if (cv_img.depth() == CV_8U) {
                frame_metadata.encoding = std::string(AssumedEncoding_1ch_u8);
            } else if (cv_img.depth() == CV_16U) {
                frame_metadata.encoding = std::string(AssumedEncoding_1ch_u16);
            } else {
                RDX_INFO_DEV(nullptr, __func__, "unknown encoding, channels={}, depth={}", cv_img.channels(), cv_img.depth());
            }
        } else if (cv_img.channels() == 3) {
            if (cv_img.depth() == CV_8U) {
                frame_metadata.encoding = std::string(AssumedEncoding_3ch_u8);
            } else if (cv_img.depth() == CV_16U) {
                frame_metadata.encoding = std::string(AssumedEncoding_3ch_u16);
            } else {
                RDX_INFO_DEV(nullptr, __func__, "unknown encoding, channels={}, depth={}", cv_img.channels(), cv_img.depth());
            }
        } else {
            RDX_INFO_DEV(nullptr, __func__, "unknown encoding, channels={}", cv_img.channels());
        }
    }
};
} // namespace redoxi_works
