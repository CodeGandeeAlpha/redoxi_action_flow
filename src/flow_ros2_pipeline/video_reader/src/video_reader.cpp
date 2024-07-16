#include "video_reader/video_reader.hpp"
#include "psg_common/psg_common.hpp"
#include "video_reader/_video_reader.hpp"
#include <cstring>
#include <memory>
#include <psg_public_msgs/msg/detail/cache_data__struct.hpp>
#include <psg_public_msgs/msg/detail/frame__struct.hpp>
#include <rclcpp/create_client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <chrono>
#include <string>
#include "vineyard/basic/ds/tensor.h"
#include <opencv2/highgui.hpp>


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
    int OpencvVideoReader::_get_current_frame_number() const {
        if (m_impl->video_capture == nullptr) {
            return -1;
        }
        if (!m_impl->video_capture->isOpened()) {
            return -1;
        }
        if (m_impl->video_capture->get(cv::CAP_PROP_FRAME_COUNT) == 0) {
            return -1;
        }

        return m_impl->video_capture->get(cv::CAP_PROP_POS_FRAMES);
    }

    void OpencvVideoReader::InitConfig::from_parameters(OpencvVideoReader* node) {
        auto logger_ = node->get_logger();

        source_file = node->get_parameter("source_file").as_string();
        source_camera_index = node->get_parameter("source_camera_index").as_int();
        start_frame_number = node->get_parameter("start_frame_number").as_int();
        end_frame_number = node->get_parameter("end_frame_number").as_int();

        RCLCPP_INFO(logger_, "[OpencvVideoReader] source_file: %s", source_file.c_str());
        RCLCPP_INFO(logger_, "[OpencvVideoReader] source_camera_index: %d", source_camera_index);
        RCLCPP_INFO(logger_, "[OpencvVideoReader] start_frame_number: %d", start_frame_number);
        RCLCPP_INFO(logger_, "[OpencvVideoReader] end_frame_number: %d", end_frame_number);
    }

    void OpencvVideoReader::RuntimeConfig::from_parameters(OpencvVideoReader* node) {
        auto logger_ = node->get_logger();
        frame_internal_ms = node->get_parameter("frame_internal_ms").as_double();
        RCLCPP_INFO(logger_, "[OpencvVideoReader] frame_internal_ms: %f", frame_internal_ms);

        image_width = node->get_parameter("image_width").as_int();
        RCLCPP_INFO(logger_, "[OpencvVideoReader] image_width: %d", image_width);
        image_height = node->get_parameter("image_height").as_int();
        RCLCPP_INFO(logger_, "[OpencvVideoReader] image_height: %d", image_height);

    }

    OpencvVideoReader::OpencvVideoReader(): rclcpp::Node("video_reader") {

        _declare_all_parameters();

        // implementation init
        m_impl = std::make_shared<OpencvVideoReaderImpl>(this);
        auto logger_ = m_impl->logger;

        m_impl->video_capture = std::make_shared<cv::VideoCapture>();
        m_impl->v6d_client = std::make_shared<vineyard::Client>();

        // v6d init
        std::string v6d_ipc_socket = "v6d_socket";
        VINEYARD_CHECK_OK(m_impl->v6d_client->Connect(v6d_ipc_socket));
        RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] v6d Connected to IPCServer: %s", v6d_ipc_socket.c_str());

        // m_timer = this->create_wall_timer(10ms, std::bind(&OpencvVideoReader::img_read, this));


        RCLCPP_INFO(logger_, "[OpencvVideoReader] construct success!");
    }

    void OpencvVideoReader::img_read() {
        auto logger_ = m_impl->logger;
        cv::Mat frame;
        auto success = m_impl->video_capture->read(frame);
        auto frame_number = _get_current_frame_number();
        RCLCPP_INFO(logger_, "[OpencvVideoReader] m_video_capture.read %d frame", frame_number);

        if (!success || frame.empty()) {
            RCLCPP_ERROR(logger_, "[OpencvVideoReader] m_video_capture.read FAILED");
        }
        else if (frame_number > m_init_config->end_frame_number && m_init_config->end_frame_number != -1) {
            RCLCPP_INFO(logger_, "[OpencvVideoReader] m_video_capture.read end_frame_number %d", m_init_config->end_frame_number);
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
                        RCLCPP_ERROR(logger_, "[OpencvVideoReader] Interrupted while waiting for the service. Exiting.");
                        return ;
                    }
                    RCLCPP_INFO(logger_, "[OpencvVideoReader] service not available, waiting again...");
                }
            }

            // 发送请求
            for (auto it: m_downstreams) {
                auto result = it.second->get_status->async_send_request(request);
                // rclcpp::spin_until_future_complete表示一定要等待服务返回
                // 如果不需要等待，可以使用rclcpp::Client::async_send_request(request, callback)里面带一个回调函数
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    if (result.get()->status == ReturnCode::REJECTED) {
                        RCLCPP_INFO(logger_, "[OpencvVideoReader] DownstreamReadyQueryService::Response is ReturnCode::REJECTED");
                    }
                    else if (result.get()->status == ReturnCode::ERROR) {
                        // TODO: shutdown node or not
                        RCLCPP_FATAL(logger_, "[OpencvVideoReader] DownstreamReadyQueryService::Response is ReturnCode::ERROR");
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

                        RCLCPP_INFO(logger_, "[OpencvVideoReader] Successfully sealed, ObjectID: %s", ObjectIDToString(id).c_str());

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

                        it.second->send_frame->async_send_goal(goal_msg, it.second->send_frame_options);
                    }
                } else { // TODO
                    RCLCPP_ERROR(logger_, "[OpencvVideoReader] call DownstreamReadyQueryService FAILED");
                }
            }
        }
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
                 std::bind(&OpencvVideoReader::img_read, this));
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