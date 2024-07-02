#include "video_reader/video_reader.hpp"

using std::placeholders::_1;

namespace FlowRos2Pipeline{
    VideoReader::VideoReader(): rclcpp::Node("video_reader"), logger_(this->get_logger()){

        // 声明参数
        this->declare_parameter<std::string>("video_path", "");
        this->declare_parameter<std::string>("frame_pub", "");
        this->declare_parameter<std::string>("frame_read_pub", "");
        this->declare_parameter<int>("start_frame_number", -1);
        this->declare_parameter<int>("end_frame_number", -1);\
        this->declare_parameter<std::string>("v6d_ipc_socket", "");

        // 获取参数
        std::string video_path = this->get_parameter("video_path").as_string();
        std::string frame_pub_topic = this->get_parameter("frame_pub").as_string();
        std::string frame_read_pub_topic = this->get_parameter("frame_read_pub").as_string();
        m_start_frame_number = this->get_parameter("start_frame_number").as_int();
        m_end_frame_number = this->get_parameter("end_frame_number").as_int();
        std::string v6d_ipc_socket = this->get_parameter("v6d_ipc_socket").as_string();
        RCLCPP_INFO(logger_, "[VideoReader] video_path: %s", video_path.c_str());
        RCLCPP_INFO(logger_, "[VideoReader] frame_pub_topic: %s", frame_pub_topic.c_str());
        RCLCPP_INFO(logger_, "[VideoReader] frame_read_pub_topic: %s", frame_read_pub_topic.c_str());
        RCLCPP_INFO(logger_, "[VideoReader] start_frame_number: %d", m_start_frame_number);
        RCLCPP_INFO(logger_, "[VideoReader] end_frame_number: %d", m_end_frame_number);


        // v6d init
        m_v6d_client = std::make_shared<vineyard::Client>();
        m_v6d_client->Connect(v6d_ipc_socket);

        // 创建publisher
        publisher_ = this->create_publisher<my_msgs::msg::Image1080p>(frame_pub_topic, 2);

        // 读取视频
        m_video_capture.open(video_path);
        if (m_start_frame_number > 1)
            m_video_capture.set(cv::CAP_PROP_POS_FRAMES, m_start_frame_number - 1);

        // 创建subscriber
        img_read_subscripter_ = this->create_subscription<std_msgs::msg::Empty>(frame_read_pub_topic,
                                        10, std::bind(&VideoReader::img_read_sub_callback, this, _1));


        RCLCPP_INFO(logger_, "[VideoReader] init success!");
    }

    void VideoReader::img_read_sub_callback(const std_msgs::msg::Empty &msg) {
        RCLCPP_INFO(logger_, "[VideoReader] img_read_sub_callback!");
        cv::Mat frame;
        auto success = m_video_capture.read(frame);
        RCLCPP_INFO(logger_, "[VideoReader] m_video_capture.read %d frame", m_frame_number);

        if (!success || frame.empty() || (m_frame_number > m_end_frame_number && m_end_frame_number != -1)) {
            RCLCPP_INFO(logger_, "[VideoReader] m_video_capture.read FAILED");
        }
        else {
            cv::resize(frame, frame, cv::Size(1920, 1080));
            auto frame_size = static_cast<size_t>(frame.step[0] * frame.rows);
            RCLCPP_INFO(logger_, "[VideoReader] frame_size %d", frame_size);

            auto image_msg = publisher_->borrow_loaned_message();

            auto msg_size = image_msg.get().data.size();
            if (frame_size != msg_size) {
                RCLCPP_ERROR(
                    logger_, "incompatible image sizes. frame %zu, msg %zu", frame_size, msg_size);
                return;
            }
            image_msg.get().is_bigendian = false;
            image_msg.get().step = m_frame_number++;
            memcpy(image_msg.get().data.data(), frame.data, frame_size);

            image_msg.get().timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::high_resolution_clock::now().time_since_epoch()).count();

            RCLCPP_INFO(logger_, "[VideoReader] publisher_->publish()");
            publisher_->publish(std::move(image_msg));
        }
    }
}