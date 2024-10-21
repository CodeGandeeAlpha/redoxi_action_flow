#pragma once

#include <redoxi_video_reader_base/VideoReaderBase.hpp>

namespace redoxi_works
{
class RandomFrameVideoReader : public RedoxiVideoReaderBase
{
  public:
    RandomFrameVideoReader(const std::string &name, const rclcpp::NodeOptions &options);

    //! output size when not specified in runtime config
    inline static const cv::Size DEFAULT_FRAME_SIZE{640, 480};

  public:
    //! Override to update runtime configuration
    int update_runtime_config(const std::shared_ptr<RuntimeConfig_t> &config) override;

  protected:
    int _read_frame(cv::Mat &frame, std::atomic<int64_t> &frame_number) override;
};
} // namespace redoxi_works
