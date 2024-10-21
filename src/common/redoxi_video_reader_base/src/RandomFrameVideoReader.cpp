#include <redoxi_video_reader_base/RandomFrameVideoReader.hpp>

namespace redoxi_works
{
RandomFrameVideoReader::RandomFrameVideoReader(const std::string &name, const rclcpp::NodeOptions &options)
    : RedoxiVideoReaderBase(name, options)
{
}

int RandomFrameVideoReader::_read_frame(cv::Mat &frame, std::atomic<int64_t> &frame_number)
{
    auto frame_size = m_runtime_config->output_image_size;
    if (frame_size.empty()) {
        RCLCPP_ERROR(get_logger(), "output_image_size is not set");
        return -1;
    }
    return 0;
}

} // namespace redoxi_works
