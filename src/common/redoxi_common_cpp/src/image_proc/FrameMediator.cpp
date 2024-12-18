#include "redoxi_common_cpp/image_proc/FrameMediator.hpp"
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <redoxi_common_cpp/ros_utils/shm_utils.hpp>
#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
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

    // read as shared first
    cv::Mat shared_cv_image;
    auto ret_shared_image = to_cv_image_shared(shared_cv_image);
    if (ret_shared_image != 0) {
        cv_image = cv::Mat();
        return ret_shared_image;
    }

    // got non empty shared image?
    if (!shared_cv_image.empty()) {
        // same encoding?
        if (source_encoding == target_encoding) {
            // no need to convert, just copy
            shared_cv_image.copyTo(cv_image);
        } else {
            // convert encoding, this is already a copy
            auto cv_img_ptr = std::make_shared<cv_bridge::CvImage>(std_msgs::msg::Header(),
                                                                   source_encoding, shared_cv_image);
            auto converted_cv_img_ptr = cv_bridge::cvtColor(cv_img_ptr, target_encoding);

            // just be safe, copy the image
            converted_cv_img_ptr->image.copyTo(cv_image);
        }
        return 0;
    } else {
        // no shared image, just return empty
        cv_image = cv::Mat();
        return 0;
    }
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

int FrameMediator::to_cv_image_shared(cv::Mat &output) const
{
    if (!m_frame_msg) {
        // no frame message, return the frame itself
        output = m_frame;
        return 0;
    }

    auto return_error = [&]() {
        output = cv::Mat();
        return -1;
    };

    auto return_ok_but_empty = [&]() {
        output = cv::Mat();
        return 0;
    };

    // have frame message, try to read the frame from it
    const auto &raw_image = m_frame_msg->raw_image;
    const auto &shm_token = m_frame_msg->shm_token;

    if (!raw_image.data.empty()) {
        // have raw data, use it
        auto dummy = std::make_shared<int>(); // required by cv_bridge
        auto cv_img = cv_bridge::toCvShare(raw_image, dummy);
        output = cv_img->image;
        return 0;
    } else {
        // try shm

        // do we have a valid shm token?
        if (!shm_utils::ShmTokenTraits::is_valid(shm_token)) {
            // no, return empty cv::Mat
            return return_ok_but_empty();
        }

        // get the default shm client for reading the frame
        auto shm_client = shared_memory::SharedMemoryFactory::get_instance().get_default_client().lock();
        if (!shm_client) {
            // no shm client, return empty cv::Mat
            RDX_WARN_DEV(nullptr, __func__, "no shm client available for service type={}",
                         shm_token.service_type);
            return return_ok_but_empty();
        }

        // check if the client is compatible with the token, return empty cv::Mat if not
        if (!shm_utils::ShmTokenTraits::is_client_and_token_compatible(shm_client.get(), shm_token)) {
            RDX_WARN_DEV(nullptr, __func__, "shm client is not compatible with the token, client.service_type={}, token.service_type={}",
                         shm_client->get_shm_config().service_type, shm_token.service_type);
            return return_error();
        }

        // now read it from client
        shared_memory::ObjectIdentifier obj_id;
        if (shm_token.object_id != shm_token.INVALID_OBJECT_ID) {
            obj_id.id = shm_token.object_id;
        } else if (!shm_token.object_key.empty()) {
            obj_id.key = shm_token.object_key;
        } else {
            RDX_WARN_DEV(nullptr, __func__, "invalid object identifier {}", obj_id.to_string());
            return return_error();
        }

        // read from shm
        cv::Mat output_cvmat;
        auto data_block = shm_client->get_data(obj_id);
        if (!data_block) {
            RDX_WARN_DEV(nullptr, __func__, "failed to read data block from shm, obj_id={}", obj_id.to_string());
            return return_error();
        }
        auto got_cvmat_from_shm = data_block->get_as_cvmat(&output_cvmat);
        if (got_cvmat_from_shm != 0) {
            RDX_WARN_DEV(nullptr, __func__, "failed to get cv::Mat from data block, obj_id={}", obj_id.to_string());
            return return_error();
        }
        output = output_cvmat;
        return 0;
    }
}

cv::Mat FrameMediator::to_cv_image_shared() const
{
    cv::Mat output;
    auto ret = to_cv_image_shared(output);
    if (ret != 0) {
        return cv::Mat();
    }
    return output;
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
            // empty, try to get it from shm
            cv::Mat shm_cv_image;
            auto ret = to_cv_image_shared(shm_cv_image);
            if (ret != 0) {
                RDX_WARN_DEV(nullptr, __func__, "failed to get cv::Mat from shm, ret={}", ret);
                return ret;
            }

            // convert shm_cv_image to image msg
            auto bridge_image = std::make_shared<cv_bridge::CvImage>(std_msgs::msg::Header(), source_encoding, shm_cv_image);
            auto converted_cv_img_ptr = cv_bridge::cvtColor(bridge_image, target_encoding);
            image_msg = *converted_cv_img_ptr->toImageMsg();
        } else {
            // we have raw image, just use it
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
                                std::optional<std::string> encoding,
                                std::optional<MsgStorageOptions_t> storage_options) const
{
    MsgStorageOptions_t _storage_options = storage_options.value_or(MsgStorageOptions_t());

    bool use_shm = false;
    std::shared_ptr<shared_memory::SharedMemoryClient> shm_client;
    std::string source_encoding = get_encoding();
    std::string target_encoding = encoding.value_or(source_encoding);

    switch (_storage_options.storage_type) {
        case MsgStorageOptions_t::StorageType::Auto:
            // do we have a client?
            shm_client = shared_memory::SharedMemoryFactory::get_instance().get_default_client().lock();
            use_shm = shm_client != nullptr; // make decision based on whether we have a client
            break;
        case MsgStorageOptions_t::StorageType::RosSerialized:
            use_shm = false;
            break;
        case MsgStorageOptions_t::StorageType::SharedMemory:
            shm_client = shared_memory::SharedMemoryFactory::get_instance().get_default_client().lock();
            use_shm = true; // no matter what, use shm
            break;
    }

    //! Convert image to frame message with optional encoding conversion and storage options
    if (!use_shm) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "not using shared memory, converting image to frame message");
        // not using shared memory, just convert it normally
        auto ret = to_image_msg(frame_msg.raw_image, encoding);
        frame_msg.metadata = m_frame_metadata;
        make_metadata_compatible(&frame_msg.metadata, frame_msg.raw_image);
        return ret;
    } else {
        // you want to use shm
        RDX_INFO_DEV(nullptr, __func__, "{}", "using shared memory");
        if (m_frame_msg) {
            // we have a frame message, is it already in shared memory and encoding is the same?
            const auto &shm_token = m_frame_msg->shm_token;
            if (shm_utils::ShmTokenTraits::is_valid(shm_token) && source_encoding == target_encoding) {
                // yes, just copy it
                RDX_INFO_DEV(nullptr, __func__, "{}", "frame message is already in shared memory, encoding is the same, just copy token");
                frame_msg = *m_frame_msg;
                return 0;
            }
        }

        // get cv mat of this frame
        RDX_INFO_DEV(nullptr, __func__, "{}", "need to convert image to cv::Mat");
        cv::Mat output_image;
        bool got_cvmat = false;
        if (source_encoding == target_encoding) {
            // same encoding, better not to copy
            RDX_INFO_DEV(nullptr, __func__, "{}", "same encoding, better not to copy");
            got_cvmat = to_cv_image_shared(output_image) == 0;
        } else {
            // different encoding, have to copy
            RDX_INFO_DEV(nullptr, __func__, "{}", "different encoding, have to copy");
            got_cvmat = to_cv_image_copy(output_image, encoding) == 0;
        }

        if (!got_cvmat) {
            // cannot get anything, return error
            RDX_INFO_DEV(nullptr, __func__, "{}", "cannot get anything, return error");
            return -1;
        } else if (output_image.empty()) {
            // empty image, no need to write to shm
            RDX_INFO_DEV(nullptr, __func__, "{}", "empty image, no need to write to shm");
            return 0;
        } else {
            // got something to write, check if we have a client
            if (!shm_client) {
                // you want to use shm but no client available, this is error
                RDX_RAISE_ERROR("[f={}] no shm client available, but requested to use shm", __func__);
            }

            // now we have a client, put the frame into it
            auto &shm_token = frame_msg.shm_token;
            shared_memory::ObjectIdentifier obj_id;
            auto data_block = shm_client->create_datablock();
            data_block->from_cvmat(output_image);
            RDX_INFO_DEV(nullptr, __func__, "putting data block into shm, image width={}, height={}, channels={}, encoding={}",
                         output_image.cols, output_image.rows, output_image.channels(), target_encoding);
            auto ret = shm_client->put_data(&obj_id, data_block.get(), nullptr, _storage_options.shm_put_options);
            if (ret != 0) {
                RDX_WARN_DEV(nullptr, __func__, "failed to put data block into shm, obj_id={}", obj_id.to_string());
                return ret;
            } else {
                RDX_INFO_DEV(nullptr, __func__, "put data block into shm, obj_id={}", obj_id.to_string());
            }

            // success, update the frame message
            // TODO: this part should be encapsulated in a function
            shm_token.object_id = obj_id.id.value();
            shm_token.object_key = obj_id.key.value_or("");
            {
                size_t size;
                data_block->get_as_bytes_ref(nullptr, &size);
                shm_token.object_size = size;
            }
            shm_token.region_key = shm_client->get_shm_config().region_key;
            shm_token.service_type = shm_client->get_shm_config().service_type;
            frame_msg.shm_token = shm_token;
            frame_msg.metadata = m_frame_metadata;
            make_metadata_compatible(&frame_msg.metadata, output_image);
            frame_msg.metadata.encoding = target_encoding;
            RDX_INFO_DEV(nullptr, __func__, "shm frame message is created, object id={}, width={}, height={}, encoding={}",
                         frame_msg.shm_token.object_id, frame_msg.metadata.width, frame_msg.metadata.height, frame_msg.metadata.encoding);
            return 0;
        }
    }
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
    const auto &raw_image = frame_msg.raw_image;

    if (!raw_image.data.empty()) {
        return is_compatible(raw_image, metadata);
    } else {
        // FIXME: this is not correct, shm image may not be compatible with metadata, leave it for now
        bool size_compatible = frame_msg.metadata.width == metadata.width && frame_msg.metadata.height == metadata.height;
        bool encoding_compatible = frame_msg.metadata.encoding == metadata.encoding;
        return size_compatible && encoding_compatible;
    }
}

} // namespace redoxi_works::image_utils
