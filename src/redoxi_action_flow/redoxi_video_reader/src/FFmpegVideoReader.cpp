#include <redoxi_video_reader/FFmpegVideoReader.hpp>
#include <filesystem>
#include <boost/asio/read_until.hpp>

namespace redoxi_works::video_readers
{

int FFmpegVideoReader::_create_logging_thread()
{
    // start reading ffmpeg output
    m_ffmpeg_logging_thread = std::make_shared<std::thread>([this]() {
        int ret = 0;
        std::string log;
        do {
            ret = _read_ffmpeg_logging_once(log);
            if (ret == 0) {
                RDX_INFO_DEV(this, __func__, "ffmpeg log: {}", log);
            }
        } while (ret == 0);
    });

    return 0;
}

int FFmpegVideoReader::_create_frame_reading_thread()
{
    // reset queue
    m_frame_queue.abort();
    m_frame_queue.clear();

    // start reading ffmpeg data
    m_ffmpeg_data_thread = std::make_shared<std::thread>([this]() {
        int ret = 0;
        do {
            ret = _read_ffmpeg_data_once();
        } while (ret == 0);
    });

    return 0;
}

int FFmpegVideoReader::_create_ffmpeg_process()
{
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    auto ffmpeg_path = init_config->ffmpeg_path;

    // open ffmpeg process and output pipe
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    m_io_context = std::make_shared<boost::asio::io_context>();
    m_ffmpeg_data_pipe = std::make_shared<boost::process::async_pipe>(*m_io_context);
    m_ffmpeg_logging_pipe = std::make_shared<boost::process::async_pipe>(*m_io_context);
    m_child_process_group = std::make_shared<boost::process::group>();

    {
        RDX_INFO_DEV(this, __func__, "opening ffmpeg process with args: {}", runtime_config->ffmpeg_args);
        std::stringstream ss;
        ss << ffmpeg_path << " ";
        for (const auto &arg : runtime_config->ffmpeg_args) {
            ss << arg << " ";
        }
        RDX_INFO_DEV(this, __func__, "complete ffmpeg command: {}", ss.str());
    }

    auto ffmpeg_process = std::make_shared<boost::process::child>(
        ffmpeg_path,
        runtime_config->ffmpeg_args,
        boost::process::std_out > *m_ffmpeg_data_pipe,
        boost::process::std_err > *m_ffmpeg_logging_pipe,
        *m_io_context,
        *m_child_process_group);

    // detach the process so that the stdout and stderr are not affected by ROS2
    // otherwise, you get no data
    // the detached process will run in the background and managed by the process group
    ffmpeg_process->detach();
    RDX_INFO_DEV(this, __func__, "{}", "process created");

    if (!m_ffmpeg_logging_pipe->is_open()) {
        RDX_RAISE_ERROR("[f={}()] Failed to open ffmpeg logging pipe", __func__);
        return -1;
    }

    if (!m_ffmpeg_data_pipe->is_open()) {
        RDX_RAISE_ERROR("[f={}()] Failed to open ffmpeg data pipe", __func__);
        return -1;
    }

    return 0;
}

int FFmpegVideoReader::_start()
{
    // call base implementation first
    {
        auto ret = RedoxiVideoReaderBase::_start();
        if (ret != 0) {
            return ret;
        }
    }

    // start ffmpeg process
    {
        RDX_INFO_DEV(this, __func__, "{}", "creating ffmpeg process");
        auto ret = _create_ffmpeg_process();
        if (ret != 0) {
            return ret;
        }
        RDX_INFO_DEV(this, __func__, "{}", "ffmpeg process created");
    }

    // start logging thread
    {
        RDX_INFO_DEV(this, __func__, "{}", "creating logging thread");
        auto ret = _create_logging_thread();
        if (ret != 0) {
            return ret;
        }
        RDX_INFO_DEV(this, __func__, "{}", "logging thread created");
    }

    // start frame reading thread
    {
        RDX_INFO_DEV(this, __func__, "{}", "creating frame reading thread");
        auto ret = _create_frame_reading_thread();
        if (ret != 0) {
            return ret;
        }
        RDX_INFO_DEV(this, __func__, "{}", "frame reading thread created");
    }

    return 0;
}


int FFmpegVideoReader::_open()
{
    // call base implementation first
    auto ret = RedoxiVideoReaderBase::_open();
    if (ret != 0) {
        return ret;
    }

    // check if ffmpeg executable exists
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    auto ffmpeg_path = init_config->ffmpeg_path;
    if (ffmpeg_path.empty() || !std::filesystem::exists(ffmpeg_path)) {
        RDX_RAISE_ERROR("[f={}()] ffmpeg executable is not specified or does not exist: {}", __func__, ffmpeg_path);
        return -1;
    }

    // frame queue is single slot
    // when reading a new frame, the old frame will be discarded, or the new frame is directly pushed to the queue
    m_frame_queue.set_capacity(1);

    return 0;
}

cv::Size FFmpegVideoReader::get_frame_size() const
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    return cv::Size(runtime_config->frame_width, runtime_config->frame_height);
}

int FFmpegVideoReader::get_frame_channels() const
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    return runtime_config->frame_channels;
}

std::string FFmpegVideoReader::get_frame_encoding() const
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    return runtime_config->frame_encoding;
}

FFmpegVideoReader::ReadFrameResult
    FFmpegVideoReader::_read_frame(SourceData_t &source_data, std::atomic<int64_t> &frame_number)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    auto src_encoding = get_frame_encoding();
    auto dst_encoding = runtime_config->output_image_encoding;

    // auto width = get_frame_size().width;
    // auto height = get_frame_size().height;
    // auto channels = get_frame_channels();
    // RDX_INFO_DEV(this, __func__, "reading frame width={}, height={}, channels={}, src_encoding={}, dst_encoding={}",
    //              width, height, channels, src_encoding, dst_encoding);

    if (!m_ffmpeg_data_pipe || !m_ffmpeg_data_pipe->is_open()) {
        RDX_RAISE_ERROR("[f={}()] ffmpeg data pipe is closed, end of video", __func__);
        return ReadFrameResult::END_OF_VIDEO;
    }

    // copy frame from reading buffer to sending buffer
    cv::Mat frame_sending;

    try {
        // wait until a frame is available
        m_frame_queue.pop(frame_sending);

        // convert encoding if necessary
        if (src_encoding != dst_encoding) {
            image_utils::convert_cv_mat_encoding(&frame_sending, frame_sending, src_encoding, dst_encoding);
        }

        // fill metadata
        auto metadata = image_utils::FrameMediator(frame_sending, dst_encoding).get_metadata();
        auto fno = _increment_frame_number_by(frame_number, 1);
        metadata.source_frame_index = fno;
        metadata.source_timestamp = rclcpp::Clock().now(); // assuming real time video
        metadata.frame_num = fno;

        // fill source data
        source_data.get_primary_frame().from_raw_data({.image = frame_sending, .metadata = metadata});

        // done
        return ReadFrameResult::OK;
    } catch (const tbb::user_abort &e) {
        //! Queue was aborted during pop operation
        RDX_INFO_DEV(this, __func__, "{}", "queue aborted, treating as end of video");
        return ReadFrameResult::END_OF_VIDEO;
    } catch (const std::exception &e) {
        RDX_RAISE_ERROR("[f={}()] failed to read frame: {}", __func__, e.what());
        return ReadFrameResult::ERROR;
    }
}

int FFmpegVideoReader::_stop()
{
    // stop child process group
    if (m_child_process_group) {
        RDX_INFO_DEV(this, __func__, "{}", "terminating child process group");
        m_child_process_group->terminate();
        m_child_process_group->wait();
    }

    if (m_ffmpeg_data_pipe) {
        RDX_INFO_DEV(this, __func__, "{}", "closing ffmpeg data pipe");
        m_ffmpeg_data_pipe->close();
    }
    if (m_ffmpeg_logging_pipe) {
        RDX_INFO_DEV(this, __func__, "{}", "closing ffmpeg logging pipe");
        m_ffmpeg_logging_pipe->close();
    }

    // clear frame queue
    RDX_INFO_DEV(this, __func__, "{}", "aborting frame queue");
    m_frame_queue.abort(); // abort any waiting threads
    m_frame_queue.clear(); // clear all remaining frames

    // join ffmpeg logging thread
    if (m_ffmpeg_logging_thread && m_ffmpeg_logging_thread->joinable()) {
        m_ffmpeg_logging_thread->join();
    }

    // join ffmpeg data thread
    if (m_ffmpeg_data_thread && m_ffmpeg_data_thread->joinable()) {
        m_ffmpeg_data_thread->join();
    }

    // cleanup pipes and shared pointers
    m_ffmpeg_data_pipe = nullptr;
    m_ffmpeg_data_thread = nullptr;
    m_ffmpeg_logging_pipe = nullptr;
    m_ffmpeg_logging_thread = nullptr;
    m_ffmpeg_process = nullptr;
    m_child_process_group = nullptr;

    // base stop
    RedoxiVideoReaderBase::_stop();

    return 0;
}

int FFmpegVideoReader::_read_ffmpeg_data_once()
{
    if (!m_ffmpeg_data_pipe || !m_ffmpeg_data_pipe->is_open()) {
        RDX_RAISE_ERROR("[f={}()] ffmpeg data pipe is closed", __func__);
        return -1;
    }

    // read data from ffmpeg
    auto expected_frame_size = get_frame_size();
    auto expected_frame_channels = get_frame_channels();
    auto expected_frame_encoding = get_frame_encoding();

    // if frame canvas is not ready, update it
    m_frame_canvas.create(expected_frame_size, CV_8UC(expected_frame_channels));

    // read data from ffmpeg
    auto buffer_size = m_frame_canvas.total() * m_frame_canvas.elemSize();
    try {
        auto num_bytes_read = boost::asio::read(*m_ffmpeg_data_pipe,
                                                boost::asio::buffer(m_frame_canvas.data, buffer_size));
        if (num_bytes_read != buffer_size) {
            RDX_RAISE_ERROR("[f={}()] failed to read ffmpeg data, expected {} bytes, but read {} bytes", __func__, buffer_size, num_bytes_read);
            return -1;
        }
    } catch (const std::exception &e) {
        RDX_RAISE_ERROR("[f={}()] failed to read ffmpeg data: {}", __func__, e.what());
        return -1;
    }

    // replace or create a new frame and push to queue
    // RDX_INFO_DEV(this, __func__, "{}", "pushing frame buffer to queue");
    cv::Mat frame_buffer;
    if (m_frame_queue.try_pop(frame_buffer)) {
        // if the queue has frame, just replace it
        m_frame_canvas.copyTo(frame_buffer);
    } else {
        // if the queue is empty, push the canvas to the queue
        m_frame_canvas.copyTo(frame_buffer);
    }
    m_frame_queue.push(frame_buffer);

    // compute the mean value of the frame
    // {
    //     auto mean_value = cv::mean(frame_buffer);
    //     RDX_INFO_DEV(this, __func__,
    //                  "read frame once, and the frame is pushed to queue, mean value: ({},{},{})",
    //                  mean_value[0], mean_value[1], mean_value[2]);
    // }

    return 0;
}

int FFmpegVideoReader::_read_ffmpeg_logging_once(std::string &out_log)
{
    if (!m_ffmpeg_logging_pipe || !m_ffmpeg_logging_pipe->is_open()) {
        RDX_RAISE_ERROR("[f={}()] ffmpeg logging pipe is closed", __func__);
        return -1;
    }

    // read logging from ffmpeg
    try {
        boost::asio::read_until(*m_ffmpeg_logging_pipe, m_ffmpeg_logging_buffer, "\n");
        std::istream is(&m_ffmpeg_logging_buffer);
        std::getline(is, out_log);
        return 0;
    } catch (const boost::system::system_error &e) {
        RDX_INFO_DEV(this, __func__, "EOF for logging: {}", e.what());
        return -1;
    } catch (const std::exception &e) {
        RDX_RAISE_ERROR("[f={}()] failed to read ffmpeg logging: {}", __func__, e.what());
        return -1;
    }
}

} // namespace redoxi_works::video_readers