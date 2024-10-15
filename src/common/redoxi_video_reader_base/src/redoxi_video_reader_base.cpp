#include "redoxi_video_reader_base/redoxi_video_reader_base.hpp"
#include "redoxi_video_reader_base/_redoxi_video_reader_base_impl.hpp"

namespace redoxi_works
{

RedoxiVideoReaderBase::RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
}

RedoxiVideoReaderBase::~RedoxiVideoReaderBase()
{
}

void RedoxiVideoReaderBase::_step()
{
    // if not started yet, do nothing
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }

    // read next frame
    cv::Mat frame;
    int ret = _read_frame(frame);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read frame, error code: %d", ret);
        return;
    }
}
} // namespace redoxi_works
