#pragma once

#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <boost/process.hpp>

// configs
namespace redoxi_works::video_readers::ffmpeg_video_reader_types
{

struct InitConfig : public RedoxiVideoReaderBase::InitConfig_t {
    // ffmpeg path
    std::string ffmpeg_path = "ffmpeg";

    // ffmpeg args, called as `ffmpeg_path ffmpeg_args[0] ... ffmpeg_args[N]`
    std::vector<std::string> ffmpeg_args;

    // image size, channels and encoding, required
    int64_t frame_width = 0;
    int64_t frame_height = 0;
    int64_t frame_channels = 0;
    std::string frame_encoding{DefaultColorImageEncoding.data()};

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(RedoxiVideoReaderBase::InitConfig_t),
        JS_MEMBER(ffmpeg_path),
        JS_MEMBER(ffmpeg_args),
        JS_MEMBER(frame_width),
        JS_MEMBER(frame_height),
        JS_MEMBER(frame_channels),
        JS_MEMBER(frame_encoding));
};

using RuntimeConfig = RedoxiVideoReaderBase::RuntimeConfig_t;

} // namespace redoxi_works::video_readers::ffmpeg_video_reader_types

namespace redoxi_works::video_readers
{
class FFmpegVideoReader : public RedoxiVideoReaderBase
{
  public:
    using InitConfig_t = ffmpeg_video_reader_types::InitConfig;
    using RuntimeConfig_t = ffmpeg_video_reader_types::RuntimeConfig;

  public:
    using RedoxiVideoReaderBase::RedoxiVideoReaderBase;
    virtual ~FFmpegVideoReader() noexcept = default;

  protected:
    int _open() override;
    int _close() override;
    ReadFrameResult _read_frame(SourceData_t &source_data, std::atomic<int64_t> &frame_number) override;
    DEFAULT_CONFIG_LOADER_IMPL(InitConfig_t, RuntimeConfig_t);

  private:
    void _async_read_ffmpeg_data();
    void _async_read_ffmpeg_logging();

  private:
    // ffmpeg process and output pipe
    std::shared_ptr<boost::process::child> m_ffmpeg_process;
    std::shared_ptr<boost::process::async_pipe> m_ffmpeg_data_pipe;
    std::shared_ptr<boost::process::async_pipe> m_ffmpeg_logging_pipe;

    std::shared_ptr<std::thread> m_ffmpeg_logging_thread;
    std::shared_ptr<std::thread> m_ffmpeg_data_thread;
    std::shared_ptr<boost::asio::io_context> m_io_context;
};
} // namespace redoxi_works::video_readers
