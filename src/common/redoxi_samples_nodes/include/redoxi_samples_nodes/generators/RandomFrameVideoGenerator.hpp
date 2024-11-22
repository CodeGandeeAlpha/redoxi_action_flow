#pragma once

#include <redoxi_video_reader/base/VideoReaderBase.hpp>

namespace redoxi_works
{
class RandomFrameVideoGenerator;
class RandomFrameVideoGeneratorRuntimeConfig : public RedoxiVideoReaderBase::RuntimeConfig_t
{
  public:
    inline static const cv::Size DEFAULT_FRAME_SIZE{512, 512};

    //! parse from node, the node must be exactly RandomFrameVideoGenerator, not its subclass
    //! @note: json_struct needs static class information, so we cannot make this a virtual method
    template <typename Node_t>
    requires std::is_same_v<Node_t, RandomFrameVideoGenerator>
    void from_node(const Node_t *node)
    {
        parse_from_node_parameters(this, node);
    }

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

  protected:
    //! Override to update runtime configuration
    int _update_runtime_config(std::shared_ptr<RedoxiVideoReaderBase::BaseRuntimeConfig_t> config) override;
    int _read_frame(SourceData_t &data, std::atomic<int64_t> &frame_number) override;
};
} // namespace redoxi_works
