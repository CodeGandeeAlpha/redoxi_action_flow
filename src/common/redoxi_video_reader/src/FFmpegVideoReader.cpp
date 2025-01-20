#include <redoxi_video_reader/FFmpegVideoReader.hpp>

namespace redoxi_works::video_readers
{

int FFmpegVideoReader::_open()
{
    // call base implementation first
    auto ret = RedoxiVideoReaderBase::_open();
    if (ret != 0) {
        return ret;
    }

    // open ffmpeg process and output pipe
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    m_ffmpeg_data_pipe = std::make_shared<boost::process::ipstream>();
    m_ffmpeg_logging_pipe = std::make_shared<boost::process::ipstream>();
    auto ffmpeg_process = std::make_shared<boost::process::child>(
        init_config->ffmpeg_path,
        init_config->ffmpeg_args,
        boost::process::std_out > *m_ffmpeg_data_pipe,
        boost::process::std_err > *m_ffmpeg_logging_pipe);

    if (!ffmpeg_process->running()) {
        RDX_RAISE_ERROR("[f={}()] Failed to open ffmpeg process: {}", __func__, init_config->ffmpeg_args);
        m_ffmpeg_data_pipe = nullptr;
        m_ffmpeg_logging_pipe = nullptr;
        return -1;
    }

    // start reading ffmpeg output
    m_ffmpeg_logging_thread = std::make_shared<std::thread>([this]() {
        // read ffmpeg logs
        std::string line;
        while (std::getline(*m_ffmpeg_logging_pipe, line)) {
            RDX_INFO_DEV(this, __func__, "ffmpeg: {}", line);
        }
    });

    return 0;
}

// TODO: implement _read_frame

int FFmpegVideoReader::_close()
{
    return 0;
}

} // namespace redoxi_works::video_readers