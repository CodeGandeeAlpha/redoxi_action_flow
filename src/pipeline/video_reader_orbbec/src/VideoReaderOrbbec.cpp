#include <video_reader_orbbec/VideoReaderOrbbec.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace redoxi_works
{
VideoReaderOrbbec::VideoReaderOrbbec(const std::string &name, const rclcpp::NodeOptions &options)
    : RedoxiVideoReaderBase(name, options)
{
}

VideoReaderOrbbec::~VideoReaderOrbbec()
{
    // closing, release orbbec pipeline
    if (m_ob_pipeline) {
        m_ob_pipeline->stop();
        m_ob_pipeline = nullptr;
        m_net_device = nullptr;
        m_ob_ctx = nullptr;
    }
}


int VideoReaderOrbbec::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
{
    int ret = RedoxiVideoReaderBase::_update_init_config(config);
    if (ret != 0) {
        return ret;
    }

    // RDX_INFO_DEV(this, __func__, false, "orbbec_net_device_ip: {}", config->orbbec_net_device_ip);

    //! 将config转换为正确的指针类型
    auto orbbec_config = std::dynamic_pointer_cast<VideoReaderOrbbecInitConfig>(config);
    if (!orbbec_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast config to VideoReaderOrbbecInitConfig", __func__);
        return -1;
    }

    RDX_ASSERT_CHECK_TRUE(!orbbec_config->orbbec_net_device_ip.empty(),
                          "[{}] orbbec_net_device_ip is empty", __func__);

    RDX_INFO_DEV(this, __func__, false, "orbbec_net_device_ip: {}", orbbec_config->orbbec_net_device_ip);

    m_ob_ctx = std::make_shared<ob::Context>();
    m_net_device = m_ob_ctx->createNetDevice(orbbec_config->orbbec_net_device_ip.c_str(), 8090);
    RDX_ASSERT_CHECK_TRUE(m_net_device != nullptr,
                          "[{}] createNetDevice failed", __func__);

    RDX_INFO_DEV(this, __func__, false, "OrbbecNetDevice name: {}", m_net_device->getDeviceInfo()->name());
    RDX_INFO_DEV(this, __func__, false, "OrbbecNetDevice serial number: {}", m_net_device->getDeviceInfo()->serialNumber());
    RDX_INFO_DEV(this, __func__, false, "OrbbecNetDevice ipAddress: {}", m_net_device->getDeviceInfo()->ipAddress());

    m_ob_pipeline = std::make_shared<ob::Pipeline>(m_net_device);
    RDX_ASSERT_CHECK_TRUE(m_ob_pipeline != nullptr,
                          "[{}] createPipeline failed", __func__);
    std::shared_ptr<ob::Config> ob_config = std::make_shared<ob::Config>();
    auto colorProfileList = m_ob_pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    auto colorProfile = colorProfileList->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>();
    if (!colorProfile) {
        colorProfile = colorProfileList->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>();
    }
    ob_config->enableStream(colorProfile);

    // Pass in the configuration and start the pipeline
    m_ob_pipeline->start(ob_config, [&](std::shared_ptr<ob::FrameSet> frameSet) {
        std::lock_guard<std::mutex> lock(m_frameset_mutex);
        m_current_frameset = frameSet;
    });

    // create format convert filter
    m_format_convert_filter = std::make_shared<ob::FormatConvertFilter>();

    return 0;
}

int VideoReaderOrbbec::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
{
    auto orbbec_config = std::dynamic_pointer_cast<VideoReaderOrbbecRuntimeConfig>(config);

    if (!orbbec_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast config to VideoReaderOrbbecRuntimeConfig", __func__);
    }

    // ensure the output image size is valid, otherwise uses default size
    if (orbbec_config->output_image_size.width <= 0 || orbbec_config->output_image_size.height <= 0) {
        RCLCPP_WARN(this->get_logger(),
                    "[%s][update_runtime_config()] Invalid output_image_size (%d, %d). Using default size.",
                    this->get_name(), orbbec_config->output_image_size.width, orbbec_config->output_image_size.height);
        orbbec_config->output_image_size = RuntimeConfig_t::DEFAULT_FRAME_SIZE;
    }

    //! Call the base class implementation first
    int ret = RedoxiVideoReaderBase::_update_runtime_config(config);
    if (ret != 0) {
        return ret;
    }


    RDX_INFO_DEV(this, __func__, false, "rotate_180: {}", orbbec_config->rotate_180);

    return 0;
}

cv::Mat VideoReaderOrbbec::_orbbec_color_to_cvmat(std::shared_ptr<ob::ColorFrame> &colorFrame)
{
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY);
    compression_params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    cv::Mat colorRawMat(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
    return colorRawMat;
}

VideoReaderOrbbec::ReadFrameResult VideoReaderOrbbec::_read_frame(SourceData_t &source_data,
                                                                  std::atomic<int64_t> &frame_number)
{
    std::shared_ptr<ob::FrameSet> frameset;
    {
        std::lock_guard<std::mutex> lock(m_frameset_mutex);
        frameset = m_current_frameset;
    }

    if (frameset == nullptr) {
        RDX_LOG_WARN(this, __func__, false, "[{}] frameset is nullptr", __func__);
        // cv::Mat black_frame = cv::Mat::zeros(1080, 1920, CV_8UC3);
        // data.set_image(black_frame);
        // data.set_frame_number(frame_number);
        // frame_number++;
        return ReadFrameResult::NO_DATA;
    }

    // get color frame frameset->colorFrame()
    auto color_frame = frameset->getFrame(OB_FRAME_COLOR);
    if (color_frame == nullptr) {
        RDX_LOG_WARN(this, __func__, false, "[{}] color_frame is nullptr", __func__);
        // cv::Mat black_frame = cv::Mat::zeros(1080, 1920, CV_8UC3);
        // data.set_image(black_frame);
        // data.set_frame_number(frame_number);
        // frame_number++;
        return ReadFrameResult::NO_DATA;
    }

    // get color frame data
    if (color_frame->format() != OB_FORMAT_RGB) {
        if (color_frame->format() == OB_FORMAT_MJPG) {
            m_format_convert_filter->setFormatConvertType(FORMAT_MJPG_TO_RGB888);
        } else if (color_frame->format() == OB_FORMAT_UYVY) {
            m_format_convert_filter->setFormatConvertType(FORMAT_UYVY_TO_RGB888);
        } else if (color_frame->format() == OB_FORMAT_YUYV) {
            m_format_convert_filter->setFormatConvertType(FORMAT_YUYV_TO_RGB888);
        } else {
            RDX_RAISE_ERROR("[{}] unsupported color frame format %d", __func__, color_frame->format());
            // cv::Mat black_frame = cv::Mat::zeros(1080, 1920, CV_8UC3);
            // data.set_image(black_frame);
            // data.set_frame_number(frame_number);
            // frame_number++;
        }
        color_frame = m_format_convert_filter->process(color_frame)->as<ob::ColorFrame>();
    }
    m_format_convert_filter->setFormatConvertType(FORMAT_RGB888_TO_BGR);

    auto _color_frame = m_format_convert_filter->process(color_frame)->as<ob::ColorFrame>();
    auto frame = _orbbec_color_to_cvmat(_color_frame);

    auto orbbec_config = std::dynamic_pointer_cast<VideoReaderOrbbecRuntimeConfig>(m_runtime_config);
    if (!orbbec_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast config to VideoReaderOrbbecRuntimeConfig", __func__);
    }

    if (orbbec_config->rotate_180) {
        cv::rotate(frame, frame, cv::ROTATE_180);
    }

    image_utils::FrameMediator fm(frame);
    redoxi_public_msgs::msg::Frame frame_msg;
    fm.to_frame_msg(frame_msg);

    SourceData_t::FrameData_t frame_data;
    frame_data.image = frame;
    frame_data.metadata = frame_msg.metadata;

    // update size info
    frame_data.metadata.width = frame.cols;
    frame_data.metadata.height = frame.rows;
    frame_data.metadata.frame_num = _increment_frame_number_by(frame_number, 1);

    source_data.set_primary_frame(frame_data);
    return ReadFrameResult::OK;
}

} // namespace redoxi_works
