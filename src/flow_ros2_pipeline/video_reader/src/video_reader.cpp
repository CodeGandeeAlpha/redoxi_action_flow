#include <cstring>
#include <future>
#include <memory>
#include <chrono>
#include <string>
#include <opencv2/core/mat.hpp>
#include <vineyard/basic/ds/tensor.h>

#include <rclcpp/create_client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rcpputils/asserts.hpp>

#include <psg_common/psg_common.hpp>
#include <psg_public_msgs/msg/detail/cache_data__struct.hpp>
#include <psg_public_msgs/msg/detail/frame__struct.hpp>

#include <video_reader/video_reader.hpp>
#include <video_reader/_video_reader.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace vineyard;

// static rclcpp::Logger& get_logger(rclcpp::Node* node)
// {
//     static auto logger = rclcpp::Logger(node->get_logger());
//     return logger;
// }

#define COMPILE_THIS
#ifdef COMPILE_THIS
namespace FlowRos2Pipeline{
    OpencvVideoReader::OpencvVideoReader(): rclcpp::Node("video_reader") {
        _declare_all_parameters();

        // implementation init
        m_impl = std::make_shared<OpencvVideoReaderImpl>(this);
        auto logger_ = m_impl->logger;

        m_impl->video_capture = std::make_shared<cv::VideoCapture>();
        m_impl->v6d_client = std::make_shared<vineyard::Client>();

        // v6d init
        std::string v6d_ipc_socket = "/var/run/vineyard.sock";
        VINEYARD_CHECK_OK(m_impl->v6d_client->Connect(v6d_ipc_socket));
        RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] v6d Connected to IPCServer: %s", v6d_ipc_socket.c_str());

        // m_timer = this->create_wall_timer(10ms, std::bind(&OpencvVideoReader::img_read, this));
        RCLCPP_INFO(logger_, "[OpencvVideoReader] construct success!");
    }

    bool OpencvVideoReader::_read_frame(cv::Mat& frame){
        ROS_ASSERT(m_impl->video_capture->isOpened(), "[OpencvVideoReader] video capture is not opened but tried to read");

        // end of required number of frames to read?
        auto frame_number = this->m_frame_number;   //convert to absolute frame number
        if(m_init_config->start_frame_number >= 0)
            frame_number += m_init_config->start_frame_number;

        if (m_init_config->end_frame_number != -1 && frame_number+1 >= m_init_config->end_frame_number) {
            RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] reached end of frame %d", m_init_config->end_frame_number);
            return false;
        }

        // read it
        auto success = m_impl->video_capture->read(frame);
        if(!success || frame.empty()){
            // end of video sequence
            return false;
        }
        this->m_frame_number += 1;
    }

    void OpencvVideoReader::_step() {
        //check status
        if(m_status_code != NodeStatusCode::STARTED){
            // nothing to do if not started
            return;
        }

        //in started mode, read frames and process them
        auto& logger = m_impl->logger;
        auto& frame = m_impl->src_frame;

        //read first or check first?
        if(m_runtime_config->read_frame_mode == RuntimeConfig::RFM_READ_ALL)
        {
            auto success = _read_frame(frame);
            if(!success || frame.empty()){
                // end of video sequence
                RCLCPP_INFO(logger,  "[OpencvVideoReader] end of video reached");
                stop();
                return;
            }
        }

        // query the downstreams to see if they are ready

        auto frame_number = _get_current_frame_number();
        RCLCPP_INFO(logger, "[OpencvVideoReader] m_video_capture.read %d frame", frame_number);

        if (!success || frame.empty()) {
            RCLCPP_ERROR(logger, "[OpencvVideoReader] m_video_capture.read FAILED");
        }
        else if (frame_number > m_init_config->end_frame_number && m_init_config->end_frame_number != -1) {
            RCLCPP_INFO(logger, "[OpencvVideoReader] m_video_capture.read end_frame_number %d", m_init_config->end_frame_number);
            m_impl->timer->cancel();
            m_status_code = NodeStatusCode::STOPPED;
        }
        else {
            cv::resize(frame, frame, cv::Size(m_runtime_config->image_width, m_runtime_config->image_height));
            // auto frame_size = static_cast<size_t>(frame.step[0] * frame.rows);

            auto request = std::make_shared<DownstreamReadyQueryService::Request>();

            for (auto it: m_downstreams) {
                while (!it.second->get_status->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(logger, "[OpencvVideoReader] Interrupted while waiting for the service. Exiting.");
                        return ;
                    }
                    RCLCPP_INFO(logger, "[OpencvVideoReader] service not available, waiting again...");
                }
                RCLCPP_INFO(logger, "[OpencvVideoReader] service available, can send request to it!");

                // auto result = it.second->get_status->async_send_request(
                //         request, std::bind(&OpencvVideoReader::status_query_callback, this, std::placeholders::_1));
                const auto& ds = it.second;

                auto result = it.second->get_status->async_send_request(request);
                auto timeout_ms = this->m_runtime_config->timeout_ms_send_frame_to_downstream;
                auto _wait_status = result.wait_for(std::chrono::milliseconds((long)timeout_ms));
                if(_wait_status == std::future_status::timeout)
                auto response = result.get();

                if (response->status == ReturnCode::REJECTED) {
                    RCLCPP_INFO(logger, "[OpencvVideoReader] DownstreamReadyQueryService::Response is ReturnCode::REJECTED");
                }
                else if (response->status == ReturnCode::ERROR) {
                    // TODO: shutdown node or not
                    RCLCPP_FATAL(logger, "[OpencvVideoReader] DownstreamReadyQueryService::Response is ReturnCode::ERROR");
                    rclcpp::shutdown();
                }
                else {
                    // 将帧存到v6d，并将id发给cache node
                    // 获取图像的尺寸
                    int height = frame.rows;
                    int width = frame.cols;
                    int ch = frame.channels();

                    // 创建 TensorBuilder，并根据图像尺寸构建 Tensor
                    TensorBuilder<uint8_t> builder(*m_impl->v6d_client, {height, width, ch});
                    auto tensor_data = builder.data();

                    // memcpy(tensor_data, frame.data, height * width * ch);

                    // 将图像数据复制到 Tensor 中
                    for (int row = 0; row < height; ++row) {
                        for (int col = 0; col < width; ++col) {
                            cv::Vec3b pixel = frame.at<cv::Vec3b>(row, col);
                            tensor_data[row * width * 3 + col * 3 + 0] = pixel[0]; // Blue
                            tensor_data[row * width * 3 + col * 3 + 1] = pixel[1]; // Green
                            tensor_data[row * width * 3 + col * 3 + 2] = pixel[2]; // Red
                        }
                    }

                    // 封存 Tensor 并持久化到 Vineyard
                    auto sealed = std::dynamic_pointer_cast<Tensor<uint8_t>>(builder.Seal(*m_impl->v6d_client));
                    VINEYARD_CHECK_OK(m_impl->v6d_client->Persist(sealed->id()));

                    ObjectID id = sealed->id();

                    RCLCPP_INFO(logger, "[OpencvVideoReader] Successfully sealed, ObjectID: %s", ObjectIDToString(id).c_str());

                    // 发送帧
                    auto goal_msg = DownstreamSendFrameAction::Goal();
                    psg_public_msgs::msg::Frame frame_msg;
                    frame_msg.frame_num = frame_number;
                    psg_public_msgs::msg::CacheData cache_data_msg;
                    cache_data_msg.id_int = id;
                    cache_data_msg.has_int_id = true;
                    cache_data_msg.id_string = ObjectIDToString(id).c_str();
                    frame_msg.cache = cache_data_msg;

                    goal_msg.frame = frame_msg;

                    downstream->send_frame->async_send_goal(goal_msg, downstream->send_frame_options);
                }
            }

        }
    }

    void OpencvVideoReader::status_query_callback(
        rclcpp::Client<DownstreamReadyQueryService>::SharedFuture future, const cv::Mat & frame, int frame_number,
        const std::shared_ptr<Downstream> & downstream) {
        auto logger_ = m_impl->logger;

        auto response = future.get();


    }

    void OpencvVideoReader::send_frame_goal_response_callback(const DownstreamSendFrameActionGoalHandle::SharedPtr & goal_handle) {
        auto logger_ = m_impl->logger;
        if (!goal_handle) {
            RCLCPP_ERROR(logger_, "Goal was rejected by server");
        } else {
            RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
        }
    }

    void OpencvVideoReader::send_frame_feedback_callback(
        DownstreamSendFrameActionGoalHandle::SharedPtr,
        const std::shared_ptr<const DownstreamSendFrameAction::Feedback> feedback)
    {

    }

    void OpencvVideoReader::send_frame_result_callback(const DownstreamSendFrameActionGoalHandle::WrappedResult & result)
    {
        auto logger_ = m_impl->logger;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(logger_, "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(logger_, "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(logger_, "Unknown result code");
                return;
        }
        std::stringstream ss;
        ss << "Result received: " << result.result->result;

        RCLCPP_INFO(logger_, "%s", ss.str().c_str());
    }


    int OpencvVideoReader::update_init_config(const std::shared_ptr<InitConfig> & config) {
        if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] update_init_config FAILED! status code is not BEFORE_INIT or CLOSED");
            return ReturnCode::ERROR;
        }

        m_init_config = config;
        // FIXME: which status code to set
        m_status_code = NodeStatusCode::INITIALIZED;
        return ReturnCode::SUCCESS;
    }

    int OpencvVideoReader::update_runtime_config(const std::shared_ptr<RuntimeConfig> & config) {
        if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] update_runtime_config FAILED! status code is not BEFORE_INIT or CLOSED");
            return ReturnCode::ERROR;
        }
        m_runtime_config = config;

        // FIXME: which status code to set
        m_status_code = NodeStatusCode::INITIALIZED;
        return ReturnCode::SUCCESS;
    }

    const std::shared_ptr<OpencvVideoReader::InitConfig> & OpencvVideoReader::get_init_config() const {
        return m_init_config;
    }

    const std::shared_ptr<OpencvVideoReader::RuntimeConfig> & OpencvVideoReader::get_runtime_config() const {
        return m_runtime_config;
    }

    int OpencvVideoReader::open() {
        if (m_status_code != NodeStatusCode::INITIALIZED) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] open FAILED! status code is not INITIALIZED");
            return ReturnCode::ERROR;
        }

        if (m_impl->v6d_client == nullptr) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] open FAILED! v6d_ipc_socket must be set");
            return ReturnCode::ERROR;
        }
        if (m_init_config->source_file.empty() && m_init_config->source_camera_index == -1) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] open FAILED! source_file and source_camera_index can not be both empty");
            return ReturnCode::ERROR;
        }

        if (m_init_config->source_camera_index != -1)
            m_impl->video_capture->open(m_init_config->source_camera_index);
        else
            m_impl->video_capture->open(m_init_config->source_file);

        if (!m_impl->video_capture->isOpened()) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] open FAILED! video capture is not opened");
            return ReturnCode::ERROR;
        }

        RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] open SUCCESS!");


        // 创建status_query_client
        std::string status_query_service = this->get_parameter("status_query_service").as_string();
        auto status_query_client_ptr_ = this->create_client<DownstreamReadyQueryService>(status_query_service);

        // 创建send_frame_client
        std::string send_frame_action = this->get_parameter("send_frame_action").as_string();
        auto send_frame_client_ptr_ = rclcpp_action::create_client<DownstreamSendFrameAction>(this, send_frame_action);

        auto send_frame_options = rclcpp_action::Client<DownstreamSendFrameAction>::SendGoalOptions();
        send_frame_options.result_callback = std::bind(&OpencvVideoReader::send_frame_result_callback, this, _1);
        send_frame_options.feedback_callback = std::bind(&OpencvVideoReader::send_frame_feedback_callback, this, _1, _2);
        send_frame_options.goal_response_callback = std::bind(&OpencvVideoReader::send_frame_goal_response_callback, this, _1);

        // downstream
        auto downstream = std::make_shared<Downstream>();
        downstream->get_status = status_query_client_ptr_;
        downstream->send_frame = send_frame_client_ptr_;
        downstream->send_frame_options = send_frame_options;
        m_downstreams["master_node"] = downstream;

        m_status_code = NodeStatusCode::OPENED;
        return ReturnCode::SUCCESS;
    }

    int OpencvVideoReader::start() {
        if (m_status_code != NodeStatusCode::OPENED) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] start FAILED! status code is not OPENED");
            return ReturnCode::ERROR;
        }

        if (!m_impl->video_capture->isOpened()) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] start FAILED! video capture is not opened");
            return ReturnCode::ERROR;
        }

        if (m_runtime_config->frame_internal_ms <= 0) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] start FAILED! RUNTIME CONFIG frame_internal_ms must be greater than 0");
            return ReturnCode::ERROR;
        }

        // set timer
        m_impl->timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(m_runtime_config->frame_internal_ms)),
                 std::bind(&OpencvVideoReader::_step, this));
    }

    int OpencvVideoReader::stop() {
        if (m_status_code != NodeStatusCode::STARTED) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] stop FAILED! status code is not STARTED");
            return ReturnCode::ERROR;
        }

        m_impl->timer->cancel();
        m_status_code = NodeStatusCode::STOPPED;
        return ReturnCode::SUCCESS;
    }

    int OpencvVideoReader::close() {
        if (m_status_code != NodeStatusCode::STOPPED) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] close FAILED! status code is not STOPPED");
            return ReturnCode::ERROR;
        }

        m_impl->video_capture->release();
        m_status_code = NodeStatusCode::CLOSED;

        m_impl = nullptr;

        // FIXME: clear downstream or not
        m_downstreams.clear();

        // FIXME: clear init config and runtime config or not
        m_init_config = nullptr;
        m_runtime_config = nullptr;


        return ReturnCode::SUCCESS;
    }

    // void OpencvVideoReader::reset() {
    //     m_video_capture.release();
    //     m_config = OpencvVideoReaderConfig();
    // }

    int OpencvVideoReader::set_image_topic_enable(bool enable) {
        m_publish_image = enable;
        return ReturnCode::SUCCESS;
    }

    std::string OpencvVideoReader::get_image_topic_name() const {
        return std::string(this->get_name()) + "/" + TOPIC_IMAGE;
    }

    int OpencvVideoReader::init(const std::shared_ptr<InitConfig>& config, const std::shared_ptr<RuntimeConfig>& runtime_config) {
        if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] init FAILED! status code is not BEFORE_INIT or CLOSED");
            return ReturnCode::ERROR;
        }

        m_init_config = config;
        m_runtime_config = runtime_config;
        m_status_code = NodeStatusCode::INITIALIZED;
        return ReturnCode::SUCCESS;
    }

    int OpencvVideoReader::get_status_code() const {
        return m_status_code;
    }

    void OpencvVideoReader::_declare_all_parameters() {
        // 声明参数
        this->declare_parameter<std::string>("source_file", "");
        this->declare_parameter<int>("source_camera_index", -1);
        this->declare_parameter<int>("start_frame_number", -1);
        this->declare_parameter<int>("end_frame_number", -1);
        this->declare_parameter<int>("image_width", -1);
        this->declare_parameter<int>("image_height", -1);

        this->declare_parameter<double>("frame_internal_ms", -1.0);

        this->declare_parameter<std::string>("status_query_service", "");
        this->declare_parameter<std::string>("send_frame_action", "");

    }
}
#endif