#include <redoxi_video_reader/VideoSourceFromUrl.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::video_readers
{

int VideoSourceFromUrl::_open()
{
    // call base implementation first
    auto ret = RedoxiVideoReaderBase::_open();
    if (ret != 0) {
        return ret;
    }

    // open file
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    auto video_capture = std::make_shared<cv::VideoCapture>(init_config->video_url);
    if (!video_capture->isOpened()) {
        RDX_INFO_DEV(this, __func__, false, "Failed to open video source from URL: {}", init_config->video_url);
        return -1;
    }
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);

    // seek to start time
    if (runtime_config->video_start_time > RuntimeConfig_t::TimeUnit_t{0}) {
        // convert to milliseconds
        auto start_time_ms = std::chrono::duration<double, std::milli>(runtime_config->video_start_time).count();
        static_assert(std::is_same_v<decltype(start_time_ms), double>);

        video_capture->set(cv::CAP_PROP_POS_MSEC, start_time_ms);
    }
    m_video_capture = video_capture;

    return 0;
}

int VideoSourceFromUrl::_close()
{
    //! Log that we're closing video source
    RDX_INFO_DEV(this, __func__, false, "{}", "Closing video source");

    //! Call base implementation first
    auto ret = RedoxiVideoReaderBase::_close();
    if (ret != 0) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Failed to close base video reader");
        return ret;
    }

    //! Close video capture
    m_video_capture.reset();
    RDX_INFO_DEV(this, __func__, false, "{}", "Video source closed successfully");

    return 0;
}

int VideoSourceFromUrl::_on_stopped()
{
    //! Log that we're triggering close
    RDX_INFO_DEV(this, __func__, false, "{}", "Triggering close on video source");

    // FIXME: the api should provide the reason why the node is stopped
    // otherwise the node will always restart even if the video source is not ready
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    if (init_config->auto_replay && rclcpp::ok()) {
        // trigger close
        close();
    }

    //! Log that close was triggered successfully
    RDX_INFO_DEV(this, __func__, false, "{}", "Close triggered successfully");

    return 0;
}

int VideoSourceFromUrl::_on_closed()
{
    //! Log that we're restarting video capture
    RDX_INFO_DEV(this, __func__, false, "{}", "Restarting video capture");

    // FIXME: broken logic, the node should know why it is closed
    // otherwise it will always restart
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    if (init_config->auto_replay && rclcpp::ok()) {
        // restart video capture
        open();

        //! Log that video capture was opened successfully
        RDX_INFO_DEV(this, __func__, false, "{}", "Video capture opened successfully");

        start();

        //! Log that video capture was started successfully
        RDX_INFO_DEV(this, __func__, false, "{}", "Video capture started successfully");
    }

    return 0;
}

const cv::VideoCapture *VideoSourceFromUrl::get_video_capture() const
{
    return m_video_capture.get();
}

cv::VideoCapture *VideoSourceFromUrl::get_video_capture()
{
    return m_video_capture.get();
}

std::string VideoSourceFromUrl::get_video_url() const
{
    auto config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    return config->video_url;
}

void VideoSourceFromUrl::set_video_url(const std::string &video_url)
{
    //! Can only set video url in closed state
    if (get_current_state().id() != NodeStatusCode::CLOSED) {
        RDX_RAISE_ERROR("[{}] Cannot set video url in non-closed state", __func__);
    }
    auto config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    config->video_url = video_url;
}

int VideoSourceFromUrl::_on_before_request_enqueue(DeliveryRequest_t &request, DeliveryPolicy_t &enqueue_policy)
{
    (void)enqueue_policy;
    auto fm = request.get_source_data().get_primary_frame().to_frame_mediator();
    RDX_INFO_DEV(this, __func__, false, "sending request with image encoding={}, in metadata={}",
                 fm.get_encoding(), fm.get_metadata().encoding);
    return 0;
}

VideoSourceFromUrl::ReadFrameResult VideoSourceFromUrl::_read_frame(SourceData_t &source_data, std::atomic<int64_t> &frame_number)
{
    // check if video capture is opened
    if (m_video_capture == nullptr || !m_video_capture->isOpened()) {
        return ReadFrameResult::ERROR;
    }

    // did we reach the user specified end time?
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    if (runtime_config->video_end_time > RuntimeConfig_t::TimeUnit_t{0}) {
        auto current_time_ms = m_video_capture->get(cv::CAP_PROP_POS_MSEC);
        auto end_time_ms = std::chrono::duration<double, std::milli>(runtime_config->video_end_time).count();
        if (current_time_ms > end_time_ms) {
            return ReadFrameResult::END_OF_VIDEO;
        }
    }

    // read frame
    cv::Mat frame;
    auto read_ok = m_video_capture->read(frame);
    if (!read_ok) {
        // failed, empty image, return end of video
        return ReadFrameResult::END_OF_VIDEO;
    }

    // current format is bgr8, convert to requested format
    auto output_image_encoding = runtime_config->output_image_encoding;
    if (output_image_encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    }
    image_utils::FrameMediator fm(frame, output_image_encoding);
    auto &primary_frame = source_data.get_primary_frame();
    primary_frame.from_raw_data({.image = frame, .metadata = fm.get_metadata()});

    // update frame number
    auto fno = _increment_frame_number_by(frame_number, 1);
    primary_frame.get_metadata().frame_num = fno;
    primary_frame.get_metadata().source_frame_index = fno;
    auto timestamp_ms = m_video_capture->get(cv::CAP_PROP_POS_MSEC);
    primary_frame.get_metadata().source_timestamp = ros2_time_msg_from_ms(timestamp_ms);

    return ReadFrameResult::OK;
}
} // namespace redoxi_works::video_readers
