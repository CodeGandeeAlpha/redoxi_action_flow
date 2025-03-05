#pragma once

#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <boost/process.hpp>
#include <boost/thread/synchronized_value.hpp>
#include <tbb/concurrent_queue.h>

// configs
namespace redoxi_works::video_readers::ffmpeg_video_reader_types
{

struct InitConfig : public RedoxiVideoReaderBase::InitConfig_t {
    // ffmpeg path
    std::string ffmpeg_path = "ffmpeg";

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(RedoxiVideoReaderBase::InitConfig_t),
        JS_MEMBER(ffmpeg_path));
};

struct RuntimeConfig : public RedoxiVideoReaderBase::RuntimeConfig_t {
    // ffmpeg args, called as `ffmpeg_path ffmpeg_args[0] ... ffmpeg_args[N]`
    std::vector<std::string> ffmpeg_args;

    // image size, channels and encoding, required
    int64_t frame_width = -1;
    int64_t frame_height = -1;
    int64_t frame_channels = 0;
    std::string frame_encoding{DefaultColorImageEncoding.data()};

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(RedoxiVideoReaderBase::RuntimeConfig_t),
        JS_MEMBER(ffmpeg_args),
        JS_MEMBER(frame_width),
        JS_MEMBER(frame_height),
        JS_MEMBER(frame_channels),
        JS_MEMBER(frame_encoding));
};

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

  public:
    cv::Size get_frame_size() const;
    int get_frame_channels() const;
    std::string get_frame_encoding() const;

  protected:
    int _open() override;  // check if ffmpeg binary exists and is executable
    int _start() override; // launch the ffmpeg process
    int _stop() override;  // stop the ffmpeg process
    ReadFrameResult _read_frame(SourceData_t &source_data, std::atomic<int64_t> &frame_number) override;
    DEFAULT_CONFIG_LOADER_IMPL(InitConfig_t, RuntimeConfig_t);

  private:
    // synchronus read ffmpeg data
    // it will always update canvas, but the frame buffer will be updated only when it is lockable
    // 0=ok, can continue reading, -1=error or eof, stop reading
    int _read_ffmpeg_data_once();

    // synchronus read ffmpeg logging
    // 0=ok, can continue reading, -1=error or eof, stop reading
    int _read_ffmpeg_logging_once(std::string &out_log);

    // creating ffmpeg process
    int _create_ffmpeg_process();

    // creating ffmpeg frame reading thread
    int _create_frame_reading_thread();

    // creating ffmpeg logging thread
    int _create_logging_thread();

  private:
    // ffmpeg process and output pipe
    std::shared_ptr<boost::process::child> m_ffmpeg_process;

    // manage the pipes and threads
    std::shared_ptr<boost::asio::io_context> m_io_context;

    // data pipe, thread, and frame cache
    std::shared_ptr<boost::process::async_pipe> m_ffmpeg_data_pipe;
    std::shared_ptr<std::thread> m_ffmpeg_data_thread;
    cv::Mat m_frame_canvas;                               // ffmpeg data will be written to this canvas
    tbb::concurrent_bounded_queue<cv::Mat> m_frame_queue; // the frames read from ffmpeg

    // logging pipe, thread, and buffer
    std::shared_ptr<boost::process::async_pipe> m_ffmpeg_logging_pipe;
    std::shared_ptr<std::thread> m_ffmpeg_logging_thread;
    boost::asio::streambuf m_ffmpeg_logging_buffer;
};
} // namespace redoxi_works::video_readers
