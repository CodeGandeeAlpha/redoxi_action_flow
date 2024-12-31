#pragma once

#include <chrono>

#include <redoxi_video_reader/base/v1/VideoReaderBase.hpp>
#include <opencv2/opencv.hpp>


namespace redoxi_works::video_readers::v1
{

class VideoSourceFromUrl;
namespace video_source_from_url_types
{
struct InitConfig : public RedoxiVideoReaderBase::InitConfig_t {
    // read video from this file
    std::string video_url;
    bool auto_replay = false;

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(RedoxiVideoReaderBase::InitConfig_t),
        JS_MEMBER(video_url),
        JS_MEMBER(auto_replay));
};

struct RuntimeConfig : public RedoxiVideoReaderBase::RuntimeConfig_t {
    // start time in default time unit, read from here
    // this is ignored for real time video source
    TimeUnit_t video_start_time{0};

    // end time in default time unit, end here, -1 means no end time
    TimeUnit_t video_end_time{-1};

    JS_OBJECT_WITH_SUPER(
        JS_SUPER(RedoxiVideoReaderBase::RuntimeConfig_t),
        JS_MEMBER(video_start_time),
        JS_MEMBER(video_end_time));
};

} // namespace video_source_from_url_types

/**
 * @brief A video reader that reads from a URL
 */
class VideoSourceFromUrl : public RedoxiVideoReaderBase
{
  public:
    using InitConfig_t = video_source_from_url_types::InitConfig;
    using RuntimeConfig_t = video_source_from_url_types::RuntimeConfig;

  public:
    VideoSourceFromUrl(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~VideoSourceFromUrl() noexcept;

    // get the video url
    std::string get_video_url() const;

    // set the video url, should be called before open
    // if you want to change it, call close() first
    void set_video_url(const std::string &video_url);

    // get the opencv video capture, so that you can query for properties like fps, width, height, etc.
    // this is not thread safe, you should only use it in stopped or closed state
    const cv::VideoCapture *get_video_capture() const;

    // get the opencv video capture, so that you can query for properties like fps, width, height, etc.
    // this is not thread safe, you should only use it in stopped or closed state
    cv::VideoCapture *get_video_capture();

  protected:
    int _open() override;
    int _close() override;
    int _on_stopped() override;
    int _on_closed() override;
    ReadFrameResult _read_frame(SourceData_t &source_data, std::atomic<int64_t> &frame_number) override;
    int _on_before_request_enqueue(DeliveryRequest_t &request, DeliveryPolicy_t &enqueue_policy) override;

  protected:
    std::shared_ptr<cv::VideoCapture> m_video_capture;
};
} // namespace redoxi_works::video_readers::v1