#pragma once

#include <redoxi_video_reader/base/VideoReaderBase.hpp>

namespace redoxi_works::video_readers
{

/**
 * @brief A video reader that reads from a file
 */
class VideoSourceFromFile : public RedoxiVideoReaderBase
{
  public:
    VideoSourceFromFile(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~VideoSourceFromFile() noexcept;
};
} // namespace redoxi_works::video_readers