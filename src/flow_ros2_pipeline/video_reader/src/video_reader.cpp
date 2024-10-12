#include <chrono>
#include <cstring>
#include <future>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <vineyard/basic/ds/tensor.h>

#include <rclcpp/create_client.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/types.hpp>
#include <rcpputils/asserts.hpp>

#include <video_reader/_video_reader.hpp>
#include <video_reader/video_reader.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;
using namespace vineyard;

// static rclcpp::Logger& get_logger(rclcpp::Node* node)
// {
//     static auto logger = rclcpp::Logger(node->get_logger());
//     return logger;
// }

#define COMPILE_THIS
#ifdef COMPILE_THIS
namespace FlowRos2Pipeline
{
bool OpencvVideoReader::_send_frame_to_downstreams(
    const MSG_Frame &frame_msg,
    bool check_downstream_ready_before_send)
{
    // service call to downstreams
    if (check_downstream_ready_before_send) {
        auto ready = _check_downstreams_ready();
        if (!ready)
            return false;
    }

    // TODO: record this into protocol
    // If v6d id is sent to downstream, and timeout, the system is in inconsistent state,
    // should be terminated, because we do not know when to delete v6d data.


    // local video need to wait for all downstreams to accept the frame
    for (auto &it : m_downstreams) {
        while (true) {
            auto goal_msg = ACT_AcceptFrame::Goal();
            goal_msg.frame = frame_msg;
            auto &ds = it.second;

            // opt.goal_response_callback = callback;
            auto res = ds->accept_frame->async_send_goal(goal_msg, ds->accept_frame_options);

            auto t = (long)m_runtime_config->timeout_ms_send_to_downstream;
            auto wait_result = res.wait_for(std::chrono::milliseconds(t));
            if (wait_result == std::future_status::ready) {
                auto task_response = res.get();
                if (task_response != nullptr) {
                    // accepted or executing
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                        task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
                        // 这里状态为这两个不一定代表成功，可能是下游还在处理中，当下游返回aborted前也会是这两个状态
                        // 所以这里需要等待下游返回成功或者aborted
                        bool is_frame_task_done = false;
                        while (true) {
                            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                                // 如果发送成功了，is_frame_task_done为true，跳出发送frame的循环
                                RCLCPP_INFO(m_impl->logger, "frame %ld success because SUCCEED", frame_msg.frame_num);
                                is_frame_task_done = true;
                                break;
                            } else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ABORTED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELED ||
                                       task_response->get_status() == rclcpp_action::GoalStatus::STATUS_CANCELING) {
                                // 如果发送失败了，is_frame_task_done为false，跳出发送frame的循环，并让外面去判断是否需要重试发送frame
                                RCLCPP_INFO(m_impl->logger, "frame %ld failed because ABORTED", frame_msg.frame_num);
                                is_frame_task_done = false;
                                break;
                            } else {
                                // 其他情况还需要等待状态变化
                                // sleep一些时间再去查询状态
                                RCLCPP_INFO(m_impl->logger, "frame %ld waiting for status change, now status is %d", frame_msg.frame_num, task_response->get_status());
                                // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                                task_response = res.get(); // 重要！！获取最新状态
                            }
                        }
                        if (is_frame_task_done) {
                            // 发送成功了，跳出发送frame的循环
                            break;
                        } else {
                            // 发送失败了，判断是否需要重发
                            if (m_runtime_config->send_goal_retry || m_init_config->video_type == VideoTypes::Local ||
                                frame_msg.signal_code == SignalCode::FLUSH || frame_msg.signal_code == SignalCode::TERMINATE) {
                                // retry
                                continue;
                            } else {
                                // failed
                                break;
                            }
                        }
                    }
                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        break;
                    }
                    // rejected
                    else {
                        // 发送失败了，判断是否需要重发
                        if (m_runtime_config->send_goal_retry || m_init_config->video_type == VideoTypes::Local ||
                            frame_msg.signal_code == SignalCode::FLUSH || frame_msg.signal_code == SignalCode::TERMINATE) {
                            // retry
                            continue;
                        } else {
                            // failed
                            break;
                        }
                    }
                } else {
                    // rejected
                    // 发送失败了，判断是否需要重发
                    if (m_runtime_config->send_goal_retry || m_init_config->video_type == VideoTypes::Local ||
                        frame_msg.signal_code == SignalCode::FLUSH || frame_msg.signal_code == SignalCode::TERMINATE) {
                        // retry
                        continue;
                    } else {
                        // failed
                        break;
                    }
                }
            } else {
            }
        }
    }
    return true;
}

uint64_t OpencvVideoReader::_add_frame_to_shared_memory(const cv::Mat &frame)
{
    int height = frame.rows;
    int width = frame.cols;
    int elem_size = frame.elemSize();
    // int ch = frame.channels();

    // 创建 TensorBuilder，并根据图像尺寸构建 Tensor
    TensorBuilder<uint8_t> builder(*m_impl->v6d_client, {height, width, elem_size});
    auto tensor_data = builder.data();

    memcpy(tensor_data, frame.data, height * width * elem_size);

    // 封存 Tensor 并持久化到 Vineyard
    auto sealed = std::dynamic_pointer_cast<Tensor<uint8_t>>(builder.Seal(*m_impl->v6d_client));
    VINEYARD_CHECK_OK(m_impl->v6d_client->Persist(sealed->id()));

    auto id = sealed->id();
    // RCLCPP_DEBUG(m_impl->logger, "[OpencvVideoReader] Successfully sealed, ObjectID_int: %ld", id);
    // RCLCPP_DEBUG(m_impl->logger, "[OpencvVideoReader] Successfully sealed, ObjectID: %s", ObjectIDToString(id).c_str());

    // auto test = m_impl->v6d_client->GetObject(id);
    // //RCLCPP_INFO(m_impl->logger, "read v6d id : %ld", test->id());

    return id;
}

OpencvVideoReader::OpencvVideoReader()
    : rclcpp::Node("video_reader")
{
    _declare_all_parameters();

    // implementation init
    m_impl = std::make_shared<OpencvVideoReaderImpl>(this);
    auto logger_ = m_impl->logger;
}

bool OpencvVideoReader::_read_frame_local(cv::Mat &frame)
{
    ROS_ASSERT(m_impl->video_capture->isOpened(), "[OpencvVideoReader] video capture is not opened but tried to read");

    // end of required number of frames to read?
    auto frame_number = this->m_frame_number; // convert to absolute frame number
    if (m_init_config->start_frame_number >= 0)
        frame_number += m_init_config->start_frame_number;

    if (m_init_config->end_frame_number != -1 && frame_number + 1 >= m_init_config->end_frame_number) {
        // RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] reached end of frame %d", m_init_config->end_frame_number);
        return false;
    }

    // read it
    auto success = m_impl->video_capture->read(frame);
    if (!success || frame.empty()) {
        // end of video sequence
        return false;
    }
    this->m_frame_number += 1;
    return true;
}

bool OpencvVideoReader::_read_frame_orbbec(cv::Mat &frame)
{
    std::shared_ptr<ob::FrameSet> frameset;
    {
        std::lock_guard<std::mutex> lock(m_impl->frameset_mutex);
        frameset = m_impl->current_frameset;
    }

    if (frameset == nullptr) {
        RCLCPP_WARN(m_impl->logger, "_read_frame_orbbec(): frameset is nullptr");
        return false;
    }

    // get color frame frameset->colorFrame()
    auto color_frame = frameset->getFrame(OB_FRAME_COLOR);
    if (color_frame == nullptr) {
        RCLCPP_ERROR(m_impl->logger, "_read_frame_orbbec(): color_frame is nullptr");
        return false;
    }

    // get color frame data
    if (color_frame->format() != OB_FORMAT_RGB) {
        if (color_frame->format() == OB_FORMAT_MJPG) {
            m_impl->format_convert_filter->setFormatConvertType(FORMAT_MJPG_TO_RGB888);
        } else if (color_frame->format() == OB_FORMAT_UYVY) {
            m_impl->format_convert_filter->setFormatConvertType(FORMAT_UYVY_TO_RGB888);
        } else if (color_frame->format() == OB_FORMAT_YUYV) {
            m_impl->format_convert_filter->setFormatConvertType(FORMAT_YUYV_TO_RGB888);
        } else {
            RCLCPP_WARN(m_impl->logger, "[OpencvVideoReader] unsupported color frame format %d", color_frame->format());
            return false;
        }
        color_frame = m_impl->format_convert_filter->process(color_frame)->as<ob::ColorFrame>();
    }
    m_impl->format_convert_filter->setFormatConvertType(FORMAT_RGB888_TO_BGR);

    auto _color_frame = m_impl->format_convert_filter->process(color_frame)->as<ob::ColorFrame>();
    frame = m_impl->orbbec_color_to_cvmat(_color_frame);
    this->m_frame_number += 1;
    return true;
}

bool OpencvVideoReader::_check_downstreams_ready()
{
    // check m_downstreams size first
    if (m_downstreams.empty())
        return false;

    // check if all downstreams can accept new frame
    for (auto it : m_downstreams) {
        auto &ds = it.second;
        auto request = std::make_shared<SRV_StatusQuery::Request>();
        auto result = ds->get_status->async_send_request(request);
        auto timeout_ms = this->m_runtime_config->timeout_ms_send_to_downstream;
        auto wait_status =
            result.wait_for(std::chrono::milliseconds((long)timeout_ms));
        if (wait_status == std::future_status::timeout) {
            // RCLCPP_DEBUG(m_impl->logger, "[OpencvVideoReader] _check_downstreams_ready TIMEOUT");
            return false;
        } else if (wait_status == std::future_status::ready) {
            // RCLCPP_DEBUG(m_impl->logger, "[OpencvVideoReader] _check_downstreams_ready READY");
            auto response = result.get();
            bool ok = response->status == ReturnCode::SUCCESS;
            if (!ok)
                return false;
        } else {
            // RCLCPP_DEBUG(m_impl->logger, "[OpencvVideoReader] _check_downstreams_ready DEFERRED");
            return false;
        }
    }
    return true;
}

bool OpencvVideoReader::_ping(const std::shared_ptr<Downstream> &ds)
{
    auto goal_msg = ACT_AcceptFrame::Goal();
    goal_msg.control_msg.control_signal = 1; // ping
    goal_msg.control_msg.control_msg = "ping";

    // opt.goal_response_callback = callback;
    auto res = ds->accept_frame->async_send_goal(goal_msg, ds->accept_frame_options);

    auto t = (long)m_runtime_config->timeout_ms_send_to_downstream;
    auto wait_result = res.wait_for(std::chrono::milliseconds(t));
    if (wait_result == std::future_status::ready) {
        auto s = res.get()->get_status();
        bool ok = false;

        // downstream accepted?
        ok |= s == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
        ok |= s == rclcpp_action::GoalStatus::STATUS_SUCCEEDED;
        ok |= s == rclcpp_action::GoalStatus::STATUS_EXECUTING;

        if (!ok)
            return false;
    } else
        return false;
    return true;
}


void OpencvVideoReader::_step()
{
    // check status
    if (m_status_code != NodeStatusCode::STARTED) {
        // nothing to do if not started
        return;
    }

    // time to read next frame?
    if (!m_impl->ready_to_read_next_frame) {
        // not yet ready to read next frame
        return;
    }

    // read frames and process them
    auto &logger = m_impl->logger;
    auto &frame = m_impl->src_frame;

    auto read_next_frame = [&]() {
        bool success = false;
        if (m_init_config->video_type == VideoTypes::OrbbecNetDevice)
            success = _read_frame_orbbec(frame);
        else
            success = _read_frame_local(frame);

        // do we have limited reading frame rate?
        if (m_runtime_config->frame_interval_ms > 0)
            m_impl->ready_to_read_next_frame = false;

        if (!success || frame.empty()) {
            // end of video sequence
            // RCLCPP_INFO(logger, "[OpencvVideoReader] end of video reached");
            if (m_init_config->video_type == VideoTypes::Local) {
                m_impl->is_video_end = true;
                stop();
            }
            return false;
        }

        // test
        if (this->m_frame_number == 200) {
            m_impl->is_video_end = true;
            stop();
            return false;
        }

        return success;
    };

    // read a frame and check if all downstreams ready
    // bool downstream_ready = false;
    if (!(m_impl->is_video_end)) {
        // if (m_runtime_config->read_frame_mode == RuntimeConfig::RFM_READ_ALL) {
        //     m_impl->read_frame_ok = read_next_frame();
        //     // if (!ok) {// read failed, but not end of video
        //     //     return;
        //     // }

        //     // downstream_ready = _check_downstreams_ready();
        //     // RCLCPP_INFO(logger, "[OpencvVideoReader] frame %ld downstream ready? %d", m_frame_number, downstream_ready);

        //     downstream_ready = _ping_downstreams(); // all downstreams are pinged

        // } else if (m_runtime_config->read_frame_mode == RuntimeConfig::RFM_READ_IF_READY) {
        //     // query first, read frame only if all downstreams can accept new frame
        //     // downstream_ready = _check_downstreams_ready();
        //     // RCLCPP_INFO(logger, "[OpencvVideoReader] frame %ld downstream ready? %d", m_frame_number, downstream_ready);

        //     downstream_ready = _ping_downstreams();

        //     // some downstream can accept this frame, read it write it to v6d and send to all downstreams
        //     if (downstream_ready) {
        //         m_impl->read_frame_ok = read_next_frame();
        //         // if (!ok)
        //         //     return;
        //     }
        // }
        m_impl->read_frame_ok = read_next_frame();
    }

    // send to downstreams
    // if (downstream_ready) {
    MSG_Frame frame_msg;
    if (!(m_impl->is_video_end)) {
        if (m_impl->read_frame_ok) {
            auto h = m_runtime_config->image_height;
            auto w = m_runtime_config->image_width;
            cv::Mat resized_frame;

            if (h > 0 && w > 0) {
                // FIXME: if h<0 or w<0, resize by preserving aspect ratio
                cv::resize(frame, m_impl->resized_frame, cv::Size(w, h));
                resized_frame = m_impl->resized_frame;
            } else
                resized_frame = frame;

            if (m_publish_image)
                _publish_frame(resized_frame);

            // cv::imshow("frame", resized_frame);

            // add frame to v6d
            auto v6d_id = _add_frame_to_shared_memory(resized_frame);
            frame_msg.cache.id_int = v6d_id;
            frame_msg.cache.has_int_id = true;
            frame_msg.cache.id_string = ObjectIDToString(v6d_id);
            frame_msg.frame_num = m_frame_number;
            frame_msg.signal_code = SignalCode::RUN;
        } else
            return;
    } else {
        frame_msg.frame_num = INT_MAX;
        frame_msg.signal_code = SignalCode::FLUSH;
    }

    // send frame to downstreams
    RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] before send frame: %ld", m_frame_number);

    // downstream actions are alreayd checked, no need to do it again
    auto frame_sent_ok = _send_frame_to_downstreams(frame_msg, false);

    RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] after send frame: %ld", m_frame_number);

    if (!frame_sent_ok) {
        // not sent to any downstream, the frame can be deleted
        auto del_ok = m_impl->v6d_client->DelData(frame_msg.cache.id_int);
        if (!del_ok.ok())
            RCLCPP_WARN(logger, "[OpencvVideoReader] failed to delete v6d data %lu", frame_msg.cache.id_int);
    }
    // }
}


int OpencvVideoReader::update_init_config(const std::shared_ptr<InitConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::OPENED &&
                   m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::STOPPED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "[OpencvVideoReader] cannot update_init_config");

    // you must either specify camera index or a video file
    ROS_ASSERT(config->source_camera_index != -1 || !config->source_file.empty(),
               "[OpencvVideoReader] source_camera_index and source_file can not be both empty");

    m_init_config = config;
    return ReturnCode::SUCCESS;
}

int OpencvVideoReader::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "[OpencvVideoReader] cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<OpencvVideoReader::InitConfig> &OpencvVideoReader::get_init_config() const
{
    return m_init_config;
}

const std::shared_ptr<OpencvVideoReader::RuntimeConfig> &OpencvVideoReader::get_runtime_config() const
{
    return m_runtime_config;
}

int OpencvVideoReader::open()
{
    // check status
    // you can open only if the node is initialized or closed
    ROS_ASSERT(m_status_code == NodeStatusCode::INITIALIZED || m_status_code == NodeStatusCode::CLOSED,
               "[OpencvVideoReader] cannot open because status code is not INITIALIZED or CLOSED");
    ROS_ASSERT(m_impl->v6d_client != nullptr, "[OpencvVideoReader] v6d_client is nullptr");
    ROS_ASSERT(m_init_config != nullptr, "[OpencvVideoReader] m_init_config is nullptr");
    ROS_ASSERT(m_init_config->video_type == VideoTypes::Local || m_init_config->video_type == VideoTypes::OrbbecNetDevice,
               "[OpencvVideoReader] video_type is not Local or OrbbecNetDevice");

    if (m_init_config->video_type == VideoTypes::Local) {
        m_impl->video_capture = std::make_shared<cv::VideoCapture>();
        if (m_init_config->source_camera_index != -1)
            m_impl->video_capture->open(m_init_config->source_camera_index);
        else {
            m_impl->video_capture->open(m_init_config->source_file);
            if (m_impl->video_capture->isOpened() && m_init_config->start_frame_number >= 0)
                m_impl->video_capture->set(cv::CAP_PROP_POS_FRAMES, m_init_config->start_frame_number);
        }


        if (!m_impl->video_capture->isOpened()) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] open FAILED! video capture is not opened");
            m_impl->video_capture = nullptr;
            return ReturnCode::ERROR;
        }
    } else if (m_init_config->video_type == VideoTypes::OrbbecNetDevice) {
        ROS_ASSERT(m_init_config->orbbec_net_device_ip != "",
                   "[OpencvVideoReader] orbbec_net_device_ip is empty");
        // connect to orbbec net device
        m_impl->ob_ctx = std::make_shared<ob::Context>();
        m_impl->net_device = m_impl->ob_ctx->createNetDevice(m_init_config->orbbec_net_device_ip.c_str(), 8090);
        ROS_ASSERT(m_impl->net_device != nullptr, "[OpencvVideoReader] net_device open failed");

        RCLCPP_WARN(m_impl->logger, "net_device name: %s", m_impl->net_device->getDeviceInfo()->name());
        RCLCPP_WARN(m_impl->logger, "net_device serial number: %s", m_impl->net_device->getDeviceInfo()->serialNumber());
        RCLCPP_WARN(m_impl->logger, "net_device ipAddress: %s", m_impl->net_device->getDeviceInfo()->ipAddress());

        m_impl->ob_pipeline = std::make_shared<ob::Pipeline>(m_impl->net_device);
        // Create Config for configuring Pipeline work
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

        // // Get the depth camera configuration list
        // auto depthProfileList = m_impl->ob_pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
        // auto depthProfile = depthProfileList->getVideoStreamProfile(1280, OB_HEIGHT_ANY, OB_FORMAT_Y16, 20);


        // if (!depthProfile) {
        //     // use default configuration
        //     depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>();
        // }
        // // enable depth stream
        // config->enableStream(depthProfile);

        // RCLCPP_WARN(m_impl->logger, "enable stream depthProfile");

        // Get the color camera configuration list
        auto colorProfileList = m_impl->ob_pipeline->getStreamProfileList(OB_SENSOR_COLOR);
        auto colorProfile = colorProfileList->getVideoStreamProfile(1280, OB_HEIGHT_ANY, OB_FORMAT_RGB, OB_FPS_ANY);
        RCLCPP_WARN(m_impl->logger, "colorProfile getVideoStreamProfile");
        if (!colorProfile) {
            // use default configuration
            colorProfile = colorProfileList->getProfile(OB_PROFILE_DEFAULT)->as<ob::VideoStreamProfile>();
        }
        // enable color stream
        config->enableStream(colorProfile);

        RCLCPP_WARN(m_impl->logger, "enable stream colorProfile");

        // Pass in the configuration and start the pipeline
        m_impl->ob_pipeline->start(config, [&](std::shared_ptr<ob::FrameSet> frameSet) {
            std::lock_guard<std::mutex> lock(m_impl->frameset_mutex);
            m_impl->current_frameset = frameSet;
        });

        // create format convert filter
        m_impl->format_convert_filter = std::make_shared<ob::FormatConvertFilter>();
    }

    // RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] open SUCCESS!");

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::OPENED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);

    // create step thread
    m_impl->step_running = true;
    m_impl->step_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok() && m_impl->step_running) {
            _step();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
        }
    });

    return ReturnCode::SUCCESS;
}

int OpencvVideoReader::start()
{
    // the node must be opened
    ROS_ASSERT(m_status_code == NodeStatusCode::OPENED,
               "[OpencvVideoReader] cannot start because status code is not OPENED");

    // start frame timer
    if (m_runtime_config->frame_interval_ms > 0) {
        // read frame every x ms
        m_impl->ready_to_read_next_frame = true; // allow reading next frame

        // setup timer to flip the flag periodically
        // the frame is read and processed in _step()
        auto t = (long)m_runtime_config->frame_interval_ms;
        auto func = [&]() { m_impl->ready_to_read_next_frame = true; };
        auto frame_timer = this->create_wall_timer(std::chrono::milliseconds(t), func);
        m_impl->frame_timer = frame_timer;
    } else {
        m_impl->frame_timer = nullptr;
    }

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STARTED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

int OpencvVideoReader::stop()
{
    // only stoppable if the node is started
    ROS_ASSERT(m_status_code == NodeStatusCode::STARTED,
               "[OpencvVideoReader] cannot stop because status code is not STARTED");

    // stop frame timer
    if (m_impl->frame_timer != nullptr) {
        m_impl->frame_timer->cancel();
        m_impl->frame_timer = nullptr;
    }

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STOPPED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

int OpencvVideoReader::close()
{
    // stop it if the node is running
    if (m_status_code == NodeStatusCode::STARTED)
        stop();

    // only valid if the node is opened or stopped
    ROS_ASSERT(m_status_code == NodeStatusCode::OPENED || m_status_code == NodeStatusCode::STOPPED,
               "[OpencvVideoReader] cannot close because status code is not OPENED or STOPPED");

    if (m_init_config->video_type == VideoTypes::Local) {
        // closing, release video capture
        m_impl->video_capture = nullptr;
    } else if (m_init_config->video_type == VideoTypes::OrbbecNetDevice) {
        // closing, release orbbec pipeline
        m_impl->ob_pipeline->stop();
        m_impl->ob_pipeline = nullptr;
        m_impl->net_device = nullptr;
        m_impl->ob_ctx = nullptr;
    }

    // closing, release v6d client
    m_impl->v6d_client->Disconnect();
    m_impl->v6d_client = nullptr;

    // reset frame number
    m_frame_number = -1;

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::CLOSED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);

    // terminate step thread
    m_impl->step_running = false;
    if (m_impl->step_thread != nullptr) {
        m_impl->step_thread->join();
        m_impl->step_thread = nullptr;
    }

    return ReturnCode::SUCCESS;
}

int OpencvVideoReader::set_image_topic_enable(bool enable)
{
    m_publish_image = enable;
    return ReturnCode::SUCCESS;
}

std::string OpencvVideoReader::get_image_topic_name() const
{
    return std::string(this->get_name()) + "/" + TOPIC_IMAGE;
}

int OpencvVideoReader::init(const std::shared_ptr<InitConfig> &config, const std::shared_ptr<RuntimeConfig> &runtime_config)
{
    // if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
    //     RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] init FAILED! status code is not BEFORE_INIT or CLOSED");
    //     return ReturnCode::ERROR;
    // }
    ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT,
               "[OpencvVideoReader] init FAILED! status code is not BEFORE_INIT");
    m_init_config = config;
    m_runtime_config = runtime_config;

    // connect to v6d
    m_impl->v6d_client = create_v6d_client();

    // create topic
    _create_image_topic();

    // setup downstreams
    _connect_to_downstreams();

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);

    // start step timer
    // auto step_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(runtime_config->step_interval_ms)),
    //         std::bind(&OpencvVideoReader::_step, this));
    // m_impl->step_timer = step_timer;

    return ReturnCode::SUCCESS;
}

void OpencvVideoReader::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "[OpencvVideoReader] m_init_config is nullptr");

    m_downstreams.clear();
    for (auto it : m_init_config->downstreams) {
        auto ds = std::make_shared<Downstream>();
        // RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] connecting to downstream %s", it.first.c_str());
        //  创建status_query_client
        {
            std::string name = it.second.status_query_service;
            auto client = this->create_client<SRV_StatusQuery>(name);
            ds->get_status = client;
            // wait until the service server is ready
            // RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] waiting for service server %s", name.c_str());
            client->wait_for_service();
            // RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] service server %s is ready", name.c_str());
        }

        // 创建accept_frame_client
        {
            std::string name = it.second.accept_frame_action;
            auto client = rclcpp_action::create_client<ACT_AcceptFrame>(this, name);
            // auto opt = rclcpp_action::Client<ACT_AcceptFrame>::SendGoalOptions();
            // opt.result_callback = std::bind(&OpencvVideoReader::send_frame_result_callback, this, _1);
            // opt.feedback_callback = std::bind(&OpencvVideoReader::send_frame_feedback_callback, this, _1, _2);
            // opt.goal_response_callback = std::bind(&OpencvVideoReader::send_frame_goal_response_callback, this, _1);
            ds->accept_frame = client;
            // ds->accept_frame_options = opt;

            // wait until the action server is ready
            // RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] waiting for action server %s", name.c_str());
            client->wait_for_action_server();
            // RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] action server %s is ready", name.c_str());
        }

        m_downstreams[it.first] = ds;
    }
}

int OpencvVideoReader::get_status_code() const
{
    return m_status_code;
}

void OpencvVideoReader::_declare_all_parameters()
{
    // 声明参数
    this->declare_parameter<std::string>("source_file", "");
    this->declare_parameter<int>("source_camera_index", -1);
    this->declare_parameter<int>("start_frame_number", -1);
    this->declare_parameter<int>("end_frame_number", -1);
    this->declare_parameter<int>("image_width", -1);
    this->declare_parameter<int>("image_height", -1);
    this->declare_parameter<std::string>("orbbec_net_device_ip", "");

    this->declare_parameter<double>("frame_interval_ms", -1.0);
    this->declare_parameter<bool>("send_goal_retry", false);
}

void OpencvVideoReader::_create_image_topic()
{
    auto topic_name = get_image_topic_name();
    m_topic_image = this->create_publisher<MSG_IMG>(topic_name, DEFAULT_IMAGE_TOPIC_QUEUE_LENGTH);
}

void OpencvVideoReader::_publish_frame(const cv::Mat &frame)
{
    MSG_IMG img_msg;
    // create sensor_msgs::msg::Image from cv::Mat
    img_msg.header.stamp = this->now();
    img_msg.header.frame_id = "camera";
    img_msg.height = frame.rows;
    img_msg.width = frame.cols;
    img_msg.encoding = "bgr8";
    img_msg.is_bigendian = false;
    img_msg.step = frame.cols * frame.elemSize();
    img_msg.data = std::vector<uint8_t>(frame.data, frame.data + frame.rows * frame.cols * frame.elemSize());
    m_topic_image->publish(img_msg);
}
} // namespace FlowRos2Pipeline
#endif