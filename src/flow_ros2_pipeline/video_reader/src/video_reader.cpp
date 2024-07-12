#include "video_reader/video_reader.hpp"
#include "video_reader/_video_reader.hpp"
#include <memory>
#include <rclcpp/create_client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <chrono>
#include "vineyard/basic/ds/tensor.h"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace vineyard;

// static rclcpp::Logger& get_logger(rclcpp::Node* node)
// {
//     static auto logger = rclcpp::Logger(node->get_logger());
//     return logger;
// }

#ifdef COMPILE_THIS
namespace FlowRos2Pipeline{
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

    void OpencvVideoReader::RuntimeConfig::from_parameter(OpencvVideoReader* node) {
        auto logger_ = node->get_logger();
        frame_internal_ms = node->get_parameter("frame_internal_ms").as_double();
        RCLCPP_INFO(logger_, "[OpencvVideoReader] frame_internal_ms: %f", frame_internal_ms);
    }

    OpencvVideoReader::OpencvVideoReader(): rclcpp::Node("video_reader") {

        // 声明参数
        this->declare_parameter<std::string>("source_file", "");
        this->declare_parameter<int>("source_camera_index", -1);
        this->declare_parameter<int>("start_frame_number", -1);
        this->declare_parameter<int>("end_frame_number", -1);
        this->declare_parameter<std::string>("v6d_ipc_socket", "");

        this->declare_parameter<double>("frame_internal_ms", -1.0);

        // implementation init
        m_impl = std::make_shared<OpencvVideoReaderImpl>(this);
        auto logger_ = m_impl->logger;

        // // 获取参数
        // std::string video_path = this->get_parameter("video_path").as_string();
        // std::string frame_pub_topic = this->get_parameter("frame_pub").as_string();
        // std::string frame_read_pub_topic = this->get_parameter("frame_read_pub").as_string();
        // m_start_frame_number = this->get_parameter("start_frame_number").as_int();
        // m_end_frame_number = this->get_parameter("end_frame_number").as_int();
        std::string v6d_ipc_socket = this->get_parameter("v6d_ipc_socket").as_string();
        // RCLCPP_INFO(logger_, "[OpencvVideoReader] video_path: %s", video_path.c_str());
        // RCLCPP_INFO(logger_, "[OpencvVideoReader] frame_pub_topic: %s", frame_pub_topic.c_str());
        // RCLCPP_INFO(logger_, "[OpencvVideoReader] frame_read_pub_topic: %s", frame_read_pub_topic.c_str());
        // RCLCPP_INFO(logger_, "[OpencvVideoReader] start_frame_number: %d", m_start_frame_number);
        // RCLCPP_INFO(logger_, "[OpencvVideoReader] end_frame_number: %d", m_end_frame_number);

        // std::string send_frame_name = this->get_parameter("send_frame_name").as_string();
        // RCLCPP_INFO(logger_, "[OpencvVideoReader] send_frame_name: %s", send_frame_name.c_str());
        // std::string status_query_name = this->get_parameter("status_query_name").as_string();
        // RCLCPP_INFO(logger_, "[OpencvVideoReader] status_query_name: %s", status_query_name.c_str());

        // v6d init
        auto v6d_ipc_socket = m_init_config.v6d_ipc_socket;
        m_impl->v6d_client = std::make_shared<vineyard::Client>();
        VINEYARD_CHECK_OK(m_impl->v6d_client->Connect(v6d_ipc_socket));
        RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] v6d Connected to IPCServer: %s", v6d_ipc_socket.c_str());


        // 创建status_query_client
        auto status_query_client_ptr_ = this->create_client<VideoReaderTypes::DownstreamReadyQueryService>(status_query_name);

        // 创建send_frame_client
        auto send_frame_client_ptr_ = rclcpp_action::create_client<VideoReaderTypes::DownstreamSendFrameAction>(this, send_frame_name);

        auto send_frame_options = rclcpp_action::Client<VideoReaderTypes::DownstreamSendFrameAction>::SendGoalOptions();
        send_frame_options.result_callback = std::bind(&OpencvVideoReader::send_frame_result_callback, this, _1);
        send_frame_options.feedback_callback = std::bind(&OpencvVideoReader::send_frame_feedback_callback, this, _1, _2);
        send_frame_options.goal_response_callback = std::bind(&OpencvVideoReader::send_frame_goal_response_callback, this, _1);

        // downstream
        VideoReaderDownstream downstream{status_query_client_ptr_, send_frame_client_ptr_, send_frame_options};
        m_downstreams["master_node"] = downstream;

        // // 打开视频
        // m_video_capture.open(video_path);
        // if (m_start_frame_number > 1)
        //     m_video_capture.set(cv::CAP_PROP_POS_FRAMES, m_start_frame_number - 1);

        // 创建subscriber
        // img_read_subscripter_ = this->create_subscription<std_msgs::msg::Empty>(frame_read_pub_topic,
        //                                 10, std::bind(&OpencvVideoReader::img_read_sub_callback, this, _1));

        // m_timer = this->create_wall_timer(10ms, std::bind(&OpencvVideoReader::img_read, this));


        RCLCPP_INFO(logger_, "[OpencvVideoReader] init success!");
    }

    void OpencvVideoReader::img_read() {
        auto logger_ = m_impl->logger;
        cv::Mat frame;
        auto success = m_video_capture.read(frame);
        RCLCPP_INFO(logger_, "[OpencvVideoReader] m_video_capture.read %d frame", m_frame_number);

        if (!success || frame.empty() || (m_frame_number > m_end_frame_number && m_end_frame_number != -1)) {
            RCLCPP_ERROR(logger_, "[OpencvVideoReader] m_video_capture.read FAILED");
        }
        else {
            cv::resize(frame, frame, cv::Size(1920, 1080));
            auto frame_size = static_cast<size_t>(frame.step[0] * frame.rows);
            RCLCPP_INFO(logger_, "[OpencvVideoReader] frame_size %ld", frame_size);

            auto request = std::make_shared<VideoReaderTypes::DownstreamReadyQueryService::Request>();

            for (auto it: m_downstreams) {
                while (!it.second.get_status->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(logger_, "[OpencvVideoReader] Interrupted while waiting for the service. Exiting.");
                        return ;
                    }
                    RCLCPP_INFO(logger_, "[OpencvVideoReader] service not available, waiting again...");
                }

            }

            // 发送请求
            for (auto it: m_downstreams) {
                auto result = it.second.get_status->async_send_request(request);
                // rclcpp::spin_until_future_complete表示一定要等待服务返回
                // 如果不需要等待，可以使用rclcpp::Client::async_send_request(request, callback)里面带一个回调函数
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    // 将帧存到v6d，并将id发给cache node
                    // 获取图像的尺寸
                    int height = frame.rows;
                    int width = frame.cols;
                    int ch = frame.channels();

                    // 创建 TensorBuilder，并根据图像尺寸构建 Tensor
                    TensorBuilder<uint8_t> builder(*m_impl->v6d_client, {height, width, ch});
                    auto tensor_data = builder.data();

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
                    auto goal_msg = VideoReaderTypes::DownstreamSendFrameAction::Goal();
                    goal_msg.frame_num = m_frame_number;

                    it.second.send_frame->async_send_goal(goal_msg, it.second.send_frame_options);

                } else { // TODO
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
                }
            }

            m_frame_number++;
        }
    }

    void OpencvVideoReader::send_frame_goal_response_callback(const VideoReaderTypes::DownstreamSendFrameActionGoalHandle::SharedPtr & goal_handle) {
        auto logger_ = m_impl->logger;
        if (!goal_handle) {
            RCLCPP_ERROR(logger_, "Goal was rejected by server");
        } else {
            RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
        }
    }

    void OpencvVideoReader::send_frame_feedback_callback(
        VideoReaderTypes::DownstreamSendFrameActionGoalHandle::SharedPtr,
        const std::shared_ptr<const VideoReaderTypes::DownstreamSendFrameAction::Feedback> feedback)
    {

    }

    void OpencvVideoReader::send_frame_result_callback(const VideoReaderTypes::DownstreamSendFrameActionGoalHandle::WrappedResult & result)
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

        // 表明可以接受下一帧
        m_can_send_frame = true;
    }


    void OpencvVideoReader::update_init_config(const InitConfig & config) {
        m_init_config = config;
    }

    void OpencvVideoReader::update_runtime_config(const RuntimeConfig & config) {
        m_runtime_config = config;
    }

    void OpencvVideoReader::open() {
        if (m_init_config.v6d_ipc_socket.empty()) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] open FAILED! v6d_ipc_socket must be set");
            return;
        }
        if (m_init_config.source_file.empty() && m_init_config.source_camera_index == -1) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] open FAILED! source_file and source_camera_index can not be both empty");
            return;
        }

        m_start_frame_number = m_init_config.start_frame_number;
        m_end_frame_number = m_init_config.end_frame_number;
        if (m_init_config.source_camera_index != -1)
            m_video_capture.open(m_init_config.source_camera_index);
        else
            m_video_capture.open(m_init_config.source_file);

    }

    void OpencvVideoReader::start() {
        if (!m_video_capture.isOpened()) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] start FAILED! video capture is not opened");
            return;
        }

        if (m_runtime_config.frame_internal_ms <= 0) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] start FAILED! frame_internal_ms must be greater than 0");
            return;
        }

        // set timer
        m_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(m_runtime_config.frame_internal_ms)),
                 std::bind(&OpencvVideoReader::img_read, this));
    }

    void OpencvVideoReader::reset() {
        m_video_capture.release();
        m_config = OpencvVideoReaderConfig();
    }
}
#endif