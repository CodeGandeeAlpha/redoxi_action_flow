#include <redoxi_samples_nodes/generators/RandomFrameVideoGenerator.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseImpl.hpp>
#include <redoxi_samples_lib/random_image.hpp>

namespace redoxi_works
{
RandomFrameVideoGenerator::RandomFrameVideoGenerator(const std::string &name, const rclcpp::NodeOptions &options)
    : RedoxiVideoReaderBase(name, options)
{
}

int RandomFrameVideoGenerator::update_runtime_config(const std::shared_ptr<RedoxiVideoReaderBase::RuntimeConfig_t> &config)
{
    // ensure the output image size is valid, otherwise uses default size
    if (config->output_image_size.width <= 0 || config->output_image_size.height <= 0) {
        RCLCPP_WARN(this->get_logger(),
                    "[%s][update_runtime_config()] Invalid output_image_size (%d, %d). Using default size.",
                    this->get_name(), config->output_image_size.width, config->output_image_size.height);
        config->output_image_size = RuntimeConfig_t::DEFAULT_FRAME_SIZE;
    }

    //! Call the base class implementation first
    int ret = RedoxiVideoReaderBase::update_runtime_config(config);
    if (ret != 0) {
        return ret;
    }
    return 0;
}

int RandomFrameVideoGenerator::_read_frame(cv::Mat &frame, std::atomic<int64_t> &frame_number)
{
    auto frame_size = m_runtime_config->output_image_size;
    if (frame_size.empty()) {
        RDX_RAISE_ERROR("[%s][_read_frame()] output_image_size is not set", this->get_name());
    }

    //! Get current ROS time as seconds from start
    auto current_time = this->now();
    auto dt_ns = (current_time - m_impl->time_node_last_started).nanoseconds();
    auto dt_ms = dt_ns / 1e6;
    RCLCPP_DEBUG(this->get_logger(), "Current time: %.2f ms from start", dt_ms);

    // generate a random frame
    auto encoding = m_runtime_config->output_image_encoding;
    cv::Mat random_frame;
    if (encoding == "mono8") {
        random_frame = cv::Mat(frame_size, CV_8UC1);
        cv::randu(random_frame, cv::Scalar(0), cv::Scalar(255));
    } else if (encoding == "mono16") {
        random_frame = cv::Mat(frame_size, CV_16UC1);
        cv::randu(random_frame, cv::Scalar(0), cv::Scalar(65535));
    } else if (encoding == "bgr8" || encoding == "rgb8") {
        random_frame = cv::Mat(frame_size, CV_8UC3);
        cv::randu(random_frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    } else {
        //! Unsupported encoding, fallback to bgr8
        RCLCPP_WARN(this->get_logger(), "Unsupported encoding: %s. Falling back to bgr8.", encoding.c_str());
        random_frame = cv::Mat(frame_size, CV_8UC3);
        cv::randu(random_frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    }
    frame = random_frame;
    frame_number++;
    return 0;
}

} // namespace redoxi_works
