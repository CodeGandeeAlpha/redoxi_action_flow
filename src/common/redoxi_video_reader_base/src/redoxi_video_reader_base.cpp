#include "redoxi_video_reader_base/redoxi_video_reader_base.hpp"

namespace redoxi_works
{

RedoxiVideoReaderBase::RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
}

RedoxiVideoReaderBase::~RedoxiVideoReaderBase()
{
}

} // namespace redoxi_works
