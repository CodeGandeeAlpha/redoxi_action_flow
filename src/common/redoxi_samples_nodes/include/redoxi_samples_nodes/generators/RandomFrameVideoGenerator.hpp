#pragma once

#include <redoxi_video_reader/base/VideoReaderBase.hpp>

namespace redoxi_works
{

class RandomFrameVideoGeneratorRuntimeConfig : public RedoxiVideoReaderBase::RuntimeConfig_t
{
  public:
    inline static const cv::Size DEFAULT_FRAME_SIZE{512, 512};

    RandomFrameVideoGeneratorRuntimeConfig()
    {
        output_image_size = DEFAULT_FRAME_SIZE;
    }
};

class RandomFrameVideoGenerator : public RedoxiVideoReaderBase
{
  public:
    RandomFrameVideoGenerator(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    using RuntimeConfig_t = RandomFrameVideoGeneratorRuntimeConfig;

  public:
    //! Override to update runtime configuration
    int update_runtime_config(std::shared_ptr<RedoxiVideoReaderBase::RuntimeConfig_t> config) override;

  protected:
    int _read_frame(SourceData_t &data, std::atomic<int64_t> &frame_number) override;
};
} // namespace redoxi_works
