#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <chrono>
#include <rclcpp_action/types.hpp>
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
    bool OpencvVideoReader::_send_frame_to_downstreams(
        const MSG_Frame& frame_msg,
        bool check_downstream_ready_before_send)
    {
        if(check_downstream_ready_before_send)
        {
            auto ready = _check_downstreams_ready();
            if(!ready) return false;
        }

        //TODO: record this into protocol
        //If v6d id is sent to downstream, and timeout, the system is in inconsistent state,
        //should be terminated, because we do not know when to delete v6d data.
        bool any_downstream_accepted_frame = false;
        for(auto& it : m_downstreams)
        {
            auto goal_msg = DownstreamAcceptFrameAction::Goal();
            goal_msg.frame = frame_msg;
            auto& ds = it.second;
            auto res = ds->accept_frame->async_send_goal(goal_msg, ds->accept_frame_options);

            auto t = (long)m_runtime_config->timeout_ms_send_frame_to_downstream;
            auto wait_result = res.wait_for(std::chrono::milliseconds(t));
            if(wait_result == std::future_status::ready){
                auto s = res.get()->get_status();
                bool ok = false;

                // downstream accepted?
                ok |= s == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
                ok |= s == rclcpp_action::GoalStatus::STATUS_SUCCEEDED;
                ok |= s == rclcpp_action::GoalStatus::STATUS_EXECUTING;

                any_downstream_accepted_frame = ok;
            }
        }

        return any_downstream_accepted_frame;
    }

    uint64_t OpencvVideoReader::_add_frame_to_shared_memory(const cv::Mat& frame){
        int height = frame.rows;
        int width = frame.cols;
        int elem_size = frame.elemSize();
        // int ch = frame.channels();

        // number of bytes in cvmat

        // 创建 TensorBuilder，并根据图像尺寸构建 Tensor
        TensorBuilder<uint8_t> builder(*m_impl->v6d_client, {height, width, elem_size});
        auto tensor_data = builder.data();

        memcpy(tensor_data, frame.data, height * width * elem_size);

        // 将图像数据复制到 Tensor 中
        // for (int row = 0; row < height; ++row) {
        //     for (int col = 0; col < width; ++col) {
        //         cv::Vec3b pixel = frame.at<cv::Vec3b>(row, col);
        //         tensor_data[row * width * 3 + col * 3 + 0] = pixel[0]; // Blue
        //         tensor_data[row * width * 3 + col * 3 + 1] = pixel[1]; // Green
        //         tensor_data[row * width * 3 + col * 3 + 2] = pixel[2]; // Red
        //     }
        // }

        // 封存 Tensor 并持久化到 Vineyard
        auto sealed = std::dynamic_pointer_cast<Tensor<uint8_t>>(builder.Seal(*m_impl->v6d_client));
        VINEYARD_CHECK_OK(m_impl->v6d_client->Persist(sealed->id()));

        auto id = sealed->id();
        RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] Successfully sealed, ObjectID: %s", ObjectIDToString(id).c_str());

        return id;
    }

    OpencvVideoReader::OpencvVideoReader(): rclcpp::Node("video_reader") {
        _declare_all_parameters();

        // implementation init
        m_impl = std::make_shared<OpencvVideoReaderImpl>(this);
        auto logger_ = m_impl->logger;
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
        return true;
    }

    bool OpencvVideoReader::_check_downstreams_ready(){
        //check if all downstreams can accept new frame
        for (auto it: m_downstreams) {
            auto& ds = it.second;
            auto request = std::make_shared<DownstreamReadyQueryService::Request>();
            auto result = ds->get_status->async_send_request(request);
            auto timeout_ms = this->m_runtime_config->timeout_ms_send_frame_to_downstream;
            auto wait_status =
                result.wait_for(std::chrono::milliseconds((long)timeout_ms));
            if(wait_status == std::future_status::timeout)
                return false;
            else{
                auto response = result.get();
                bool ok = response->status == ReturnCode::SUCCESS;
                if(!ok) return false;
            }
        }
        return true;
    }


    void OpencvVideoReader::_step() {
        //check status
        if(m_status_code != NodeStatusCode::STARTED){
            // nothing to do if not started
            return;
        }

        //time to read next frame?
        if(!m_impl->ready_to_read_next_frame){
            // not yet ready to read next frame
            return;
        }

        //read frames and process them
        auto& logger = m_impl->logger;
        auto& frame = m_impl->src_frame;

        auto read_next_frame = [&](){
            auto success = _read_frame(frame);
            if(!success || frame.empty()){
                // end of video sequence
                RCLCPP_INFO(logger,  "[OpencvVideoReader] end of video reached");
                stop();
                return false;
            }
            return success;
        };

        //read a frame and check if all downstreams ready
        bool downstream_ready = false;
        if(m_runtime_config->read_frame_mode == RuntimeConfig::RFM_READ_ALL)
        {
            auto ok = read_next_frame();
            if(!ok) return;

            downstream_ready = _check_downstreams_ready();
        }
        else if(m_runtime_config->read_frame_mode == RuntimeConfig::RFM_READ_IF_READY)
        {
            //query first, read frame only if all downstreams can accept new frame
            downstream_ready = _check_downstreams_ready();

            //some downstream can accept this frame, read it write it to v6d and send to all downstreams
            if(downstream_ready)
            {
                auto ok = read_next_frame();
                if(!ok) return;
            }
        }

        // send to downstreams
        if(downstream_ready)
        {
            auto h = m_runtime_config->image_height;
            auto w = m_runtime_config->image_width;
            cv::Mat resized_frame;

            if(h>0 && w>0)
            {
                //FIXME: if h<0 or w<0, resize by preserving aspect ratio
                cv::resize(frame, m_impl->resized_frame,cv::Size(w, h));
                resized_frame = m_impl->resized_frame;
            }
            else
                resized_frame = frame;

            //add frame to v6d
            auto v6d_id = _add_frame_to_shared_memory(resized_frame);

            //send frame to downstreams
            MSG_Frame frame_msg;
            frame_msg.frame_num = m_frame_number;
            frame_msg.cache.id_int = v6d_id;
            frame_msg.cache.has_int_id = true;

            //downstream actions are alreayd checked, no need to do it again
            auto frame_sent_ok = _send_frame_to_downstreams(frame_msg, false);

            if(!frame_sent_ok)
            {
                //not sent to any downstream, the frame can be deleted
                auto del_ok = m_impl->v6d_client->DelData(v6d_id);
                if(!del_ok.ok())
                    RCLCPP_WARN(logger, "[OpencvVideoReader] failed to delete v6d data %lu", v6d_id);
            }
        }
    }


    int OpencvVideoReader::update_init_config(const std::shared_ptr<InitConfig> & config) {
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

    int OpencvVideoReader::update_runtime_config(const std::shared_ptr<RuntimeConfig> & config) {
        ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                    m_status_code != NodeStatusCode::BEFORE_INIT,
                "[OpencvVideoReader] cannot update_runtime_config");

        m_runtime_config = config;
        return ReturnCode::SUCCESS;
    }

    const std::shared_ptr<OpencvVideoReader::InitConfig> & OpencvVideoReader::get_init_config() const {
        return m_init_config;
    }

    const std::shared_ptr<OpencvVideoReader::RuntimeConfig> & OpencvVideoReader::get_runtime_config() const {
        return m_runtime_config;
    }

    int OpencvVideoReader::open() {
        // check status
        // you can open only if the node is initialized or closed
        ROS_ASSERT(m_status_code == NodeStatusCode::INITIALIZED || m_status_code == NodeStatusCode::CLOSED,
                "[OpencvVideoReader] cannot open because status code is not INITIALIZED or CLOSED");
        ROS_ASSERT(m_impl->v6d_client != nullptr, "[OpencvVideoReader] v6d_client is nullptr");

        m_impl->video_capture = std::make_shared<cv::VideoCapture>();
        if (m_init_config->source_camera_index != -1)
            m_impl->video_capture->open(m_init_config->source_camera_index);
        else
        {
            m_impl->video_capture->open(m_init_config->source_file);
            if(m_impl->video_capture->isOpened() && m_init_config->start_frame_number >= 0)
                m_impl->video_capture->set(cv::CAP_PROP_POS_FRAMES, m_init_config->start_frame_number);
        }


        if (!m_impl->video_capture->isOpened()) {
            RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] open FAILED! video capture is not opened");
            m_impl->video_capture = nullptr;
            return ReturnCode::ERROR;
        }

        RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] open SUCCESS!");

        m_status_code = NodeStatusCode::OPENED;
        return ReturnCode::SUCCESS;
    }

    int OpencvVideoReader::start() {
        // the node must be opened
        ROS_ASSERT(m_status_code == NodeStatusCode::OPENED,
                "[OpencvVideoReader] cannot start because status code is not OPENED");

        // start frame timer
        if(m_runtime_config->frame_interval_ms > 0){
            // read frame every x ms
            auto t = (long)m_runtime_config->frame_interval_ms;
            auto func = [&](){m_impl->ready_to_read_next_frame = true;};
            auto frame_timer = this->create_wall_timer(std::chrono::milliseconds(t),func);
            m_impl->frame_timer = frame_timer;
        }else{
            m_impl->frame_timer = nullptr;
        }

        m_status_code = NodeStatusCode::STARTED;
    }

    int OpencvVideoReader::stop() {
        // only stoppable if the node is started
        ROS_ASSERT(m_status_code == NodeStatusCode::STARTED,
                "[OpencvVideoReader] cannot stop because status code is not STARTED");

        m_status_code = NodeStatusCode::STOPPED;
        return ReturnCode::SUCCESS;
    }

    int OpencvVideoReader::close() {
        // stop it if the node is running
        if(m_status_code == NodeStatusCode::STARTED)
            stop();

        //only valid if the node is opened or stopped
        ROS_ASSERT(m_status_code == NodeStatusCode::OPENED || m_status_code == NodeStatusCode::STOPPED,
                "[OpencvVideoReader] cannot close because status code is not OPENED or STOPPED");

        // closing, release video capture
        m_impl->video_capture = nullptr;

        // reset frame number
        m_frame_number = -1;

        m_status_code = NodeStatusCode::CLOSED;
        return ReturnCode::SUCCESS;
    }

    int OpencvVideoReader::set_image_topic_enable(bool enable) {
        m_publish_image = enable;
        return ReturnCode::SUCCESS;
    }

    std::string OpencvVideoReader::get_image_topic_name() const {
        return std::string(this->get_name()) + "/" + TOPIC_IMAGE;
    }

    int OpencvVideoReader::init(const std::shared_ptr<InitConfig>& config, const std::shared_ptr<RuntimeConfig>& runtime_config) {
        // if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
        //     RCLCPP_ERROR(m_impl->logger, "[OpencvVideoReader] init FAILED! status code is not BEFORE_INIT or CLOSED");
        //     return ReturnCode::ERROR;
        // }
        ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT,
                "[OpencvVideoReader] cannot init");
        m_init_config = config;
        m_runtime_config = runtime_config;

        // connect to v6d
        m_impl->v6d_client = create_v6d_client();

        // create topic
        _create_image_topic();

        // setup downstreams
        _connect_to_downstreams();

        //TODO: log all state transitions for debugging
        m_status_code = NodeStatusCode::INITIALIZED;

        // start step timer
        auto step_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(runtime_config->step_interval_ms)),
                std::bind(&OpencvVideoReader::_step, this));
        m_impl->step_timer = step_timer;

        return ReturnCode::SUCCESS;
    }

    void OpencvVideoReader::_connect_to_downstreams(){
        ROS_ASSERT(m_init_config != nullptr, "[OpencvVideoReader] m_init_config is nullptr");

        m_downstreams.clear();
        for(auto it: m_init_config->downstreams){
            auto ds = std::make_shared<Downstream>();
            RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] connecting to downstream %s", it.first.c_str());
            // 创建status_query_client
            {
                std::string name = it.second.status_query_service;
                auto client = this->create_client<DownstreamReadyQueryService>(name);
                ds->get_status = client;
            }

            // 创建accept_frame_client
            {
                std::string name = it.second.accept_frame_action;
                auto client = rclcpp_action::create_client<DownstreamAcceptFrameAction>(this, name);
                // auto opt = rclcpp_action::Client<DownstreamAcceptFrameAction>::SendGoalOptions();
                // opt.result_callback = std::bind(&OpencvVideoReader::send_frame_result_callback, this, _1);
                // opt.feedback_callback = std::bind(&OpencvVideoReader::send_frame_feedback_callback, this, _1, _2);
                // opt.goal_response_callback = std::bind(&OpencvVideoReader::send_frame_goal_response_callback, this, _1);
                ds->accept_frame = client;
                // ds->accept_frame_options = opt;

                // wait until the action server is ready
                RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] waiting for action server %s", name.c_str());
                client->wait_for_action_server();
                RCLCPP_INFO(m_impl->logger, "[OpencvVideoReader] action server %s is ready", name.c_str());
            }

            m_downstreams[it.first] = ds;
        }
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

    void OpencvVideoReader::_create_image_topic(){
        auto topic_name = get_image_topic_name();
        m_topic_image = this->create_publisher<sensor_msgs::msg::Image>(topic_name, DEFAULT_IMAGE_TOPIC_QUEUE_LENGTH);
    }
}
#endif