#include "redoxi_video_reader_base/redoxi_video_reader_base.hpp"
#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <redoxi_common_cpp/redoxi_v6d.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>

namespace redoxi_works
{
namespace ap = async_processor;
namespace mytypes = RedoxiVideoReaderBaseTypes;

class RedoxiVideoReaderImpl
{
  public:
    virtual ~RedoxiVideoReaderImpl()
    {
    }
    RedoxiVideoReaderImpl(RedoxiVideoReaderBase *node)
        : logger(node->get_logger())
    {
    }

    rclcpp::Logger logger;
    std::shared_ptr<vineyard::Client> v6d_client;
    std::shared_ptr<cv::VideoCapture> video_capture;

    rclcpp::TimerBase::SharedPtr step_timer;
    rclcpp::TimerBase::SharedPtr frame_timer;
    bool ready_to_read_next_frame = true;
    bool read_frame_ok = false;
    bool is_video_end = false;
    cv::Mat src_frame;     // last read frame, avoid creating cv::Mat object every time
    cv::Mat resized_frame; // resized frame

    // flags to convert async call to sync call
    std::vector<bool> frame_sent_flags;
    std::shared_ptr<std::thread> step_thread;
    bool step_running = false; // for stopping the step thread

    // token for reading a new frame every x-milliseconds
    std::shared_ptr<RosTimeToken_ms> read_frame_token;

    // frame delivery node
    std::shared_ptr<
        ap::SingleBufferExecNode<
            mytypes::FrameDeliveryTask,
            mytypes::FrameDeliveryTask>>
        frame_delivery_node;
    std::shared_ptr<tbb::flow::graph> frame_delivery_graph;
};

RedoxiVideoReaderBase::RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
    // in subclass, you should declare your own parameters
    _declare_all_parameters();
    m_impl = std::make_shared<RedoxiVideoReaderImpl>(this);

    // FIXME: moved these to init() later
    _init_frame_delivery_tasks();
}

void RedoxiVideoReaderBase::_init_frame_delivery_tasks()
{
    // create graph and node
    m_impl->frame_delivery_graph = std::make_shared<tbb::flow::graph>();
    auto &g = *m_impl->frame_delivery_graph;

    m_impl->frame_delivery_node = std::make_shared<
        ap::SingleBufferExecNode<
            mytypes::FrameDeliveryTask,
            mytypes::FrameDeliveryTask>>(g);
    auto &node = *m_impl->frame_delivery_node;

    // set node params
    node.set_input_data_buffer_size(1);
    node.set_preserve_order(true);

    // sync mode, all functions are executed in the graph
    node.set_is_async(false);

    // setup work function
    // TODO:
}

RedoxiVideoReaderBase::~RedoxiVideoReaderBase()
{
}

void RedoxiVideoReaderBase::_declare_all_parameters()
{
    // parameters
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

void RedoxiVideoReaderBase::_step()
{
    // if not started yet, do nothing
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }

    // read next frame, if token is ready
    {
        DummyTimeToken token;
        if (m_impl->read_frame_token->try_pop_token(token)) {
            // time to read a new frame
            cv::Mat frame;
            int ret = _read_frame(frame);
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to read frame, error code: %d", ret);
            } else {
                // send frame to downstreams
                _send_frame_to_downstreams(frame);
            }
        }
    }

} // namespace redoxi_works
