#pragma once

#include <redoxi_video_reader/base/VideoReaderBase.hpp>

namespace redoxi_works::video_readers
{

/**
 * @brief A video reader that reads from a file
 */
class FileVideoReader : public RedoxiVideoReaderBase
{
  public:
    FileVideoReader(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~FileVideoReader() noexcept;
};
} // namespace redoxi_works::video_readers