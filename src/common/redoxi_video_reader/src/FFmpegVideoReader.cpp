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

FFmpegVideoReader::ReadFrameResult
    FFmpegVideoReader::_read_frame(SourceData_t &source_data, std::atomic<int64_t> &frame_number)
{
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    auto width = init_config->frame_width;
    auto height = init_config->frame_height;
    auto channels = init_config->frame_channels;
    auto src_encoding = init_config->frame_encoding;

    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    auto dst_encoding = runtime_config->output_image_encoding;

    // read raw frame from ffmpeg, assuming 8bit per-channel video
    cv::Mat raw_frame(height, width, CV_8UC(channels));
    if (!m_ffmpeg_data_pipe->read(reinterpret_cast<char *>(raw_frame.data), raw_frame.total() * raw_frame.elemSize())) {
        // no more data to read, end of video
        return ReadFrameResult::END_OF_VIDEO;
    }

    // convert encoding if necessary
    if (src_encoding != dst_encoding) {
        image_utils::convert_cv_mat_encoding(&raw_frame, raw_frame, src_encoding, dst_encoding);
    }

    // fill metadata
    auto metadata = image_utils::FrameMediator(raw_frame, dst_encoding).get_metadata();
    auto fno = _increment_frame_number_by(frame_number, 1);
    metadata.source_frame_index = fno;
    metadata.source_timestamp = rclcpp::Clock().now(); // assuming real time video
    metadata.frame_num = fno;

    // fill source data
    source_data.get_primary_frame().from_raw_data({.image = raw_frame, .metadata = metadata});

    // done
    return ReadFrameResult::OK;
}

int FFmpegVideoReader::_close()
{
    // stop ffmpeg process
    if (m_ffmpeg_process && m_ffmpeg_process->running()) {
        m_ffmpeg_process->terminate();
        m_ffmpeg_process->wait();
    }

    // join ffmpeg logging thread
    if (m_ffmpeg_logging_thread && m_ffmpeg_logging_thread->joinable()) {
        m_ffmpeg_logging_thread->join();
    }

    // cleanup pipes and shared pointers
    m_ffmpeg_data_pipe = nullptr;
    m_ffmpeg_logging_pipe = nullptr;
    m_ffmpeg_process = nullptr;
    m_ffmpeg_logging_thread = nullptr;

    return 0;
}

} // namespace redoxi_works::video_readers