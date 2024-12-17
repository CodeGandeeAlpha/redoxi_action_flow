#include "redoxi_common_cpp/image_proc/FrameMediator.hpp"
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <unordered_set>
#include <sensor_msgs/image_encodings.hpp>

static const std::unordered_set<std::string> allowed_3ch_encodings = {
    sensor_msgs::image_encodings::RGB8,
    sensor_msgs::image_encodings::BGR8,
    sensor_msgs::image_encodings::RGB16,
    sensor_msgs::image_encodings::BGR16,
    sensor_msgs::image_encodings::YUV422,
};

static const std::unordered_set<std::string> allowed_1ch_encodings = {
    sensor_msgs::image_encodings::MONO8,
    sensor_msgs::image_encodings::MONO16,
};

namespace redoxi_works::image_utils
{

//! Construct from metadata only, assuming the frame is all black
FrameMediator::FrameMediator(const Metadata_t &frame_metadata)
    : m_frame_metadata(frame_metadata)
{
}

//! Construct from cv::Mat and metadata
FrameMediator::FrameMediator(const cv::Mat &frame, const Metadata_t &frame_metadata)
    : m_frame(frame), m_frame_metadata(frame_metadata)
{
    if (!is_compatible(frame, frame_metadata)) {
        RDX_RAISE_ERROR("[f={}] frame is not compatible with metadata", __func__);
    }
}

//! Construct from cv::Mat and encoding
FrameMediator::FrameMediator(const cv::Mat &frame, const std::string &encoding)
    : m_frame(frame)
{
    if (encoding.empty()) {
        RDX_RAISE_ERROR("[f={}] encoding is empty", __func__);
    }

    make_metadata_compatible(&m_frame_metadata, frame);
    m_frame_metadata.encoding = encoding;
}

//! Construct from cv::Mat with default metadata
FrameMediator::FrameMediator(const cv::Mat &frame)
    : m_frame(frame)
{
    make_metadata_compatible(&m_frame_metadata, frame);
}

//! Construct from frame message
FrameMediator::FrameMediator(const FrameMsg_t *frame_msg)
    : m_frame_msg(frame_msg)
{
    // metadata should be compatible with the frame message
    if (!is_compatible(*m_frame_msg, m_frame_msg->metadata)) {
        RDX_RAISE_ERROR("[f={}] frame is not compatible with metadata", __func__);
    }
    m_frame_metadata = m_frame_msg->metadata;
}

int FrameMediator::to_cv_image_copy(cv::Mat &cv_image, std::optional<std::string> encoding) const
{
    std::string target_encoding = encoding.value_or(get_encoding());
    std::string source_encoding = get_encoding();

    if (m_frame_msg) {
        const auto &raw_image = m_frame_msg->raw_image;
        if (raw_image.data.empty()) {
            cv_image = cv::Mat();
        } else {
            auto cv_img = cv_bridge::toCvCopy(raw_image, target_encoding);
            cv_image = cv_img->image;
        }
    } else if (m_frame.empty()) {
        cv_image = cv::Mat();
    } else {
        if (source_encoding == target_encoding) {
            RDX_INFO_DEV(nullptr, __func__, "source encoding={}, target encoding={}, no conversion",
                         source_encoding, target_encoding);
            cv_image = m_frame.clone();
        } else {
            RDX_INFO_DEV(nullptr, __func__, "source encoding={}, target encoding={}, convert encoding",
                         source_encoding, target_encoding);
            auto cv_img_ptr = std::make_shared<cv_bridge::CvImage>(std_msgs::msg::Header(),
                                                                   source_encoding, m_frame);
            auto converted_cv_img_ptr = cv_bridge::cvtColor(cv_img_ptr, target_encoding);
            cv_image = converted_cv_img_ptr->image;
        }
    }
    return 0;
}

cv::Mat FrameMediator::to_cv_image_copy(std::optional<std::string> encoding) const
{
    cv::Mat cv_image;
    auto ret = to_cv_image_copy(cv_image, encoding);
    if (ret != 0) {
        return cv::Mat();
    }
    return cv_image;
}

cv::Mat FrameMediator::to_cv_image_shared() const
{
    if (m_frame_msg) {
        const auto &raw_image = m_frame_msg->raw_image;

        if (raw_image.data.empty()) {
            return cv::Mat();
        }

        auto dummy = std::make_shared<int>();
        auto cv_img = cv_bridge::toCvShare(raw_image, dummy);
        return cv_img->image;
    } else {
        return m_frame;
    }
}

int FrameMediator::to_image_msg(sensor_msgs::msg::Image &image_msg,
                                std::optional<std::string> encoding) const
{
    //! Get source and target encodings, using source encoding as default if no target specified
    auto source_encoding = get_encoding();
    auto target_encoding = encoding.value_or(source_encoding);

    if (m_frame_msg) {
        //! If we have a frame message, work with the raw image data
        const auto &raw_image = m_frame_msg->raw_image;

        if (raw_image.data.empty()) {
            //! For empty raw image, just copy as-is
            image_msg = raw_image;
            image_msg.encoding = target_encoding;
        } else {
            if (source_encoding == target_encoding) {
                //! If encodings match, copy raw image directly
                image_msg = raw_image;
            } else {
                //! Convert encoding using cv_bridge if needed
                auto cv_img_ptr = cv_bridge::toCvCopy(raw_image, target_encoding);
                image_msg = *cv_img_ptr->toImageMsg();
            }
        }
    } else {
        //! If we have a cv::Mat instead of frame message
        if (source_encoding == target_encoding) {
            //! If encodings match, convert directly to ROS message
            cv_bridge::CvImage cv_img(std_msgs::msg::Header(), source_encoding, m_frame);
            cv_img.toImageMsg(image_msg);
        } else {
            //! Convert encoding using cv_bridge if needed
            auto cv_img_ptr = std::make_shared<cv_bridge::CvImage>(
                std_msgs::msg::Header(), source_encoding, m_frame);
            auto converted_cv_img_ptr = cv_bridge::cvtColor(cv_img_ptr, target_encoding);
            converted_cv_img_ptr->toImageMsg(image_msg);
        }
    }
    return 0;
}

int FrameMediator::to_frame_msg(redoxi_public_msgs::msg::Frame &frame_msg,
                                std::optional<std::string> encoding) const
{
    //! Convert image to frame message with optional encoding conversion
    auto ret = to_image_msg(frame_msg.raw_image, encoding);
    frame_msg.metadata = m_frame_metadata;
    make_metadata_compatible(&frame_msg.metadata, frame_msg.raw_image);
    return ret;
}

// Getter implementations
int64_t FrameMediator::get_device_type() const
{
    return m_frame_metadata.device_tag.device_type;
}
int64_t FrameMediator::get_device_index() const
{
    return m_frame_metadata.device_tag.device_index;
}
int64_t FrameMediator::get_width() const
{
    return m_frame_metadata.width;
}
int64_t FrameMediator::get_height() const
{
    return m_frame_metadata.height;
}
int64_t FrameMediator::get_frame_number() const
{
    return m_frame_metadata.frame_num;
}
const std::string &FrameMediator::get_encoding() const
{
    return m_frame_metadata.encoding;
}
int64_t FrameMediator::get_source_frame_index() const
{
    return m_frame_metadata.source_frame_index;
}

std::chrono::microseconds FrameMediator::get_source_timestamp_flat() const
{
    auto secs = std::chrono::microseconds(m_frame_metadata.source_timestamp.sec * 1000000);
    auto nsecs = std::chrono::microseconds(m_frame_metadata.source_timestamp.nanosec);
    return secs + nsecs;
}

builtin_interfaces::msg::Time FrameMediator::get_source_timestamp() const
{
    return m_frame_metadata.source_timestamp;
}

const FrameMediator::Metadata_t &FrameMediator::get_metadata() const
{
    return m_frame_metadata;
}
const cv::Mat &FrameMediator::_get_frame() const
{
    return m_frame;
}

// Private helper methods
void FrameMediator::make_metadata_compatible(Metadata_t *frame_metadata, const cv_bridge::CvImage &cv_img)
{
    frame_metadata->width = cv_img.image.cols;
    frame_metadata->height = cv_img.image.rows;
    frame_metadata->encoding = cv_img.encoding;
}

void FrameMediator::make_metadata_compatible(Metadata_t *frame_metadata, const sensor_msgs::msg::Image &image_msg)
{
    frame_metadata->width = image_msg.width;
    frame_metadata->height = image_msg.height;
    frame_metadata->encoding = image_msg.encoding;
}

void FrameMediator::make_metadata_compatible(Metadata_t *frame_metadata, const cv::Mat &cv_img)
{
    frame_metadata->width = cv_img.cols;
    frame_metadata->height = cv_img.rows;

    if (cv_img.channels() == 1) {
        // skip it if already has compatible encoding
        if (allowed_1ch_encodings.count(frame_metadata->encoding) > 0) {
            return;
        }

        if (cv_img.depth() == CV_8U) {
            frame_metadata->encoding = std::string(AssumedEncoding_1ch_u8);
        } else if (cv_img.depth() == CV_16U) {
            frame_metadata->encoding = std::string(AssumedEncoding_1ch_u16);
        } else {
            RDX_INFO_DEV(nullptr, __func__, "unknown encoding, channels={}, depth={}",
                         cv_img.channels(), cv_img.depth());
        }
    } else if (cv_img.channels() == 3) {
        // skip it if already has compatible encoding
        if (allowed_3ch_encodings.count(frame_metadata->encoding) > 0) {
            return;
        }

        if (cv_img.depth() == CV_8U) {
            frame_metadata->encoding = std::string(AssumedEncoding_3ch_u8);
        } else if (cv_img.depth() == CV_16U) {
            frame_metadata->encoding = std::string(AssumedEncoding_3ch_u16);
        } else {
            RDX_INFO_DEV(nullptr, __func__, "unknown encoding, channels={}, depth={}",
                         cv_img.channels(), cv_img.depth());
        }
    } else {
        RDX_INFO_DEV(nullptr, __func__, "unknown encoding, channels={}", cv_img.channels());
    }
}

bool FrameMediator::is_compatible(const cv::Mat &mat, const Metadata_t &metadata)
{
    if (mat.empty())
        return metadata == Metadata_t();

    bool size_compatible = mat.cols == metadata.width && mat.rows == metadata.height;
    if (!size_compatible) {
        RDX_INFO_DEV(nullptr, __func__, "frame size is not compatible with metadata, frame size={}, metadata size={}",
                     mat.cols, mat.rows, metadata.width, metadata.height);
        return false;
    }

    bool encoding_compatible = false;
    if (mat.channels() == 1) {
        encoding_compatible = allowed_1ch_encodings.count(metadata.encoding) > 0;
    } else if (mat.channels() == 3) {
        encoding_compatible = allowed_3ch_encodings.count(metadata.encoding) > 0;
    }

    if (!encoding_compatible) {
        RDX_INFO_DEV(nullptr, __func__, "frame encoding is not compatible with metadata, frame channels={}, metadata encoding={}",
                     mat.channels(), metadata.encoding);
    }
    return size_compatible && encoding_compatible;
}

bool FrameMediator::is_compatible(const ImageMsg_t &image_msg, const Metadata_t &metadata)
{
    if (image_msg.data.empty())
        return metadata == Metadata_t();

    bool size_compatible = image_msg.width == metadata.width && image_msg.height == metadata.height;
    if (!size_compatible) {
        RDX_INFO_DEV(nullptr, __func__, "image size is not compatible with metadata, image size={}, metadata size={}",
                     image_msg.width, image_msg.height, metadata.width, metadata.height);
        return false;
    }

    bool encoding_compatible = image_msg.encoding == metadata.encoding;
    if (!encoding_compatible) {
        RDX_INFO_DEV(nullptr, __func__, "image encoding is not compatible with metadata, image encoding={}, metadata encoding={}",
                     image_msg.encoding, metadata.encoding);
    }
    return size_compatible && encoding_compatible;
}

bool FrameMediator::is_compatible(const FrameMsg_t &frame_msg, const Metadata_t &metadata)
{
    return is_compatible(frame_msg.raw_image, metadata);
}

} // namespace redoxi_works::image_utils
