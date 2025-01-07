#pragma once

#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Error.hpp>

namespace redoxi_works
{

struct VideoReaderOrbbecInitConfig : public RedoxiVideoReaderBase::InitConfig_t {
    std::string orbbec_net_device_ip;

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(RedoxiVideoReaderBase::InitConfig_t),
                         JS_MEMBER(orbbec_net_device_ip));
};

struct VideoReaderOrbbecRuntimeConfig : public RedoxiVideoReaderBase::RuntimeConfig_t {
  public:
    inline static const cv::Size DEFAULT_FRAME_SIZE{1920, 1080};
    bool rotate_180{false};

    VideoReaderOrbbecRuntimeConfig()
    {
        output_image_size = DEFAULT_FRAME_SIZE;
    }

    // json serialize
    JS_OBJECT_WITH_SUPER(JS_SUPER(RedoxiVideoReaderBase::RuntimeConfig_t),
                         JS_MEMBER(rotate_180));
};

class VideoReaderOrbbec : public RedoxiVideoReaderBase
{
  public:
    using InitConfig_t = VideoReaderOrbbecInitConfig;
    using RuntimeConfig_t = VideoReaderOrbbecRuntimeConfig;
    using BaseNode_t = RedoxiVideoReaderBase;
    using BaseNode_t::BaseNode_t;
    ~VideoReaderOrbbec();

  protected:
    //! Override to update init configuration
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> config) override;

    //! Override to update runtime configuration
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config) override;

    DEFAULT_CONFIG_LOADER_IMPL(InitConfig_t, RuntimeConfig_t);

  protected:
    virtual ReadFrameResult _read_frame(SourceData_t &source_data,
                                        std::atomic<int64_t> &frame_number) override;

  protected:
    cv::Mat _orbbec_color_to_cvmat(std::shared_ptr<ob::ColorFrame> &colorFrame);

  protected:
    // orbbec pipeline
    std::shared_ptr<ob::Context> m_ob_ctx;
    std::shared_ptr<ob::Device> m_net_device;
    std::shared_ptr<ob::Pipeline> m_ob_pipeline;
    std::shared_ptr<ob::FrameSet> m_current_frameset;
    std::mutex m_frameset_mutex;
    std::shared_ptr<ob::FormatConvertFilter> m_format_convert_filter;
};
} // namespace redoxi_works
