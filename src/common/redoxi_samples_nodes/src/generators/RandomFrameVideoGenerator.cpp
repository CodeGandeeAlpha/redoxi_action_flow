#include <redoxi_samples_nodes/generators/RandomFrameVideoGenerator.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseImpl.hpp>
#include <redoxi_samples_lib/random_image.hpp>
#include <redoxi_common_cpp/image_proc/ImageStamper.hpp>

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

    //! Generate a random frame with the UUID text
    cv::Mat random_frame;
    random_image_with_text(random_frame, frame_size);

    frame = random_frame;
    frame_number++;
    return 0;
}

} // namespace redoxi_works
