#pragma once

#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseImpl.hpp>

namespace redoxi_works
{
class SimpleActionGenerator : public RedoxiVideoReaderBase
{
  public:
    SimpleActionGenerator(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  protected:
    void _step() override;

    //! Implement _read_frame method
    int _read_frame(cv::Mat &frame,
                    std::atomic<int64_t> &frame_number) override;
};
} // namespace redoxi_works