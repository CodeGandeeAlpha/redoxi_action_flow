#include "redoxi_video_reader_base/redoxi_video_reader_base.hpp"
#include "redoxi_video_reader_base/redoxi_video_reader_types.hpp"
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
        if (frame_delivery_graph) {
            frame_delivery_graph->wait_for_all();
        }
    }
    RedoxiVideoReaderImpl(RedoxiVideoReaderBase *node)
        : logger(node->get_logger())
    {
        this->ros_node = node;
    }

    // create vineyard client
    int _init_v6d_client()
    {
        // get parameters
        auto v6d_socket_name = ros_node->declare_parameter<std::string>(RosParams::Keys::v6d_socket_name, "");
        if (v6d_socket_name.empty()) {
            //! Vineyard socket name is not set
            RCLCPP_WARN(logger, "[%s][_init_v6d_client()] Vineyard socket name is not set, using default socket", ros_node->get_name());
        }

        // create vineyard client
        v6d_client = std::make_shared<VineyardClient>();

        // connect to vineyard server
        auto ret = v6d_client->connect(v6d_socket_name);
        if (!ret) {
            RCLCPP_ERROR(logger, "[%s][_init_v6d_client()] Failed to connect to vineyard server", ros_node->get_name());
            v6d_client = nullptr;
            return -1;
        }
        return 0;
    }

    RedoxiVideoReaderBase *ros_node = nullptr;

    rclcpp::Logger logger;
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
    using FrameDeliveryNode_t = ap::SingleBufferExecNode<mytypes::FrameDeliveryTask, mytypes::FrameDeliveryTask>;
    std::shared_ptr<FrameDeliveryNode_t> frame_delivery_node;
    std::shared_ptr<tbb::flow::graph> frame_delivery_graph;

    // vineyard client, used to interact with vineyard shared memory
    std::shared_ptr<VineyardClient> v6d_client;
};

void RedoxiVideoReaderBase::set_publish_image(bool enable)
{
    m_publish_image = enable;
}

RedoxiVideoReaderBase::RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
    // declare parameters, should be called in constructor before everything else
    _declare_all_parameters();

    // in subclass, you should declare your own parameters
    m_impl = std::make_shared<RedoxiVideoReaderImpl>(this);

    // init vineyard client
    int ret = m_impl->_init_v6d_client();
    if (ret != 0) {
        RCLCPP_WARN(this->get_logger(), "[%s][constructor()] Failed to connect to vineyard server, continue without vineyard", this->get_name());
    }
}

void RedoxiVideoReaderBase::_set_status_code(int status_code)
{
    m_status_code = status_code;
}

int RedoxiVideoReaderBase::init(const std::shared_ptr<InitConfig_t> &config,
                                const std::shared_ptr<RuntimeConfig_t> &runtime_config)
{
    // must be in BEFORE_INIT status
    //! Ensure node is in BEFORE_INIT status
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::BEFORE_INIT,
                          "[{}][init()] Invalid node status for initialization: {}", this->get_name(), m_status_code);

    // both config and runtime config must be set
    RDX_ASSERT_CHECK_TRUE(config != nullptr,
                          "[{}][init()] Init config is not set", this->get_name());
    RDX_ASSERT_CHECK_TRUE(runtime_config != nullptr,
                          "[{}][init()] Runtime config is not set", this->get_name());

    // init vineyard client
    int ret = m_impl->_init_v6d_client();
    if (ret != 0) {
        RCLCPP_WARN(this->get_logger(), "[%s][init()] Failed to connect to vineyard server, continue without vineyard", this->get_name());
    }

    //! create frame delivery tasks
    {
        int ret = _init_frame_delivery_tasks();
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "[%s][update_init_config()] Failed to initialize frame delivery tasks", this->get_name());
            return ret;
        }
    }

    // create debug topics
    _create_debug_topics();

    // set init_config
    {
        auto ret = update_init_config(config);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "[%s][init()] Failed to update init config", this->get_name());
            return ret;
        }
    }

    // set runtime_config
    {
        auto ret = update_runtime_config(runtime_config);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "[%s][init()] Failed to update runtime config", this->get_name());
            return ret;
        }
    }

    return 0;
}

int RedoxiVideoReaderBase::update_init_config(const std::shared_ptr<InitConfig_t> &config)
{
    //! Ensure the node is in CLOSED or BEFORE_INIT status
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::CLOSED ||
                              m_status_code == NodeStatusCode::BEFORE_INIT,
                          "[{}][update_init_config()] Invalid node status for updating init config: {}",
                          this->get_name(), m_status_code);

    //! Ensure the new config is not null
    RDX_ASSERT_CHECK_TRUE(config != nullptr,
                          "[{}][update_init_config()] New init config is null", this->get_name());

    //! Update the init config
    m_init_config = config;

    //! Reconnect to downstreams with the new config
    auto ret = _connect_to_downstreams();
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "[%s][update_init_config()] Failed to connect to downstreams, error code: %d", this->get_name(), ret);
        return ret;
    }

    //! Set status code to closed
    _set_status_code(NodeStatusCode::CLOSED);

    return 0;
}

int RedoxiVideoReaderBase::update_runtime_config(const std::shared_ptr<RuntimeConfig_t> &config)
{
    //! Ensure the node is in CLOSED or STOPPED
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::CLOSED ||
                              m_status_code == NodeStatusCode::STOPPED,
                          "[{}][update_runtime_config()] Invalid node status for updating runtime config: {}",
                          this->get_name(), m_status_code);

    //! Ensure the new config is not null
    RDX_ASSERT_CHECK_TRUE(config != nullptr,
                          "[{}][update_runtime_config()] New runtime config is null", this->get_name());

    //! Update the runtime config
    m_runtime_config = config;

    // create timer based token for reading a new frame every x-milliseconds
    {
        auto frame_interval_ms = (int64_t)m_runtime_config->frame_interval_ms;
        m_impl->read_frame_token = std::make_shared<RosTimeToken_ms>(this, std::chrono::milliseconds(frame_interval_ms));
    }

    //! Apply any necessary changes based on the new runtime config
    //! This might involve updating internal parameters or reconfiguring components

    RCLCPP_INFO(this->get_logger(), "[%s][update_runtime_config()] Runtime config updated successfully", this->get_name());

    return 0;
}

const std::shared_ptr<RedoxiVideoReaderBase::InitConfig_t> &RedoxiVideoReaderBase::get_init_config() const
{
    //! Return the current init configuration
    return m_init_config;
}


void RedoxiVideoReaderBase::_create_debug_topics()
{
    //! Create publisher for debug image topic if enabled
    std::string topic_name = get_publish_image_topic_name();
    int queue_size = get_publish_image_queue_size();

    m_topic_image = this->create_publisher<sensor_msgs::msg::Image>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(queue_size)).reliable());

    RCLCPP_DEBUG(this->get_logger(),
                 "[%s][_create_debug_topics()] Created debug image topic: %s",
                 this->get_name(),
                 topic_name.c_str());
}

int RedoxiVideoReaderBase::open()
{
    //! Ensure the node is in CLOSED state
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::CLOSED,
                          "[{}][open()] Invalid node status for opening: {}", this->get_name(), m_status_code);

    // do subclass work first, to avoid side effect on failure
    auto ret = _open();
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "[%s][open()] Failed to open node", this->get_name());
        return ret;
    }

    //! Set status code to opened
    _set_status_code(NodeStatusCode::OPENED);

    return 0;
}

int RedoxiVideoReaderBase::start()
{
    //! Ensure the node is in OPENED state
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::OPENED ||
                              m_status_code == NodeStatusCode::STOPPED,
                          "[{}][start()] Invalid node status for starting: {}", this->get_name(), m_status_code);

    // do subclass work first, to avoid side effect on failure
    auto ret = _start();
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "[%s][start()] Failed to start node", this->get_name());
        return ret;
    }

    // create a timer based token for reading a new frame every x-milliseconds
    {
        auto frame_interval_ms = (int64_t)m_runtime_config->frame_interval_ms;
        m_impl->read_frame_token->start(std::chrono::milliseconds(frame_interval_ms));
    }


    //! Set status code to STARTED
    _set_status_code(NodeStatusCode::STARTED);

    return 0;
}

int RedoxiVideoReaderBase::stop()
{
    //! Ensure the node is in STARTED state
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::STARTED,
                          "[{}][stop()] Invalid node status for stopping: {}", this->get_name(), m_status_code);

    // do subclass work first, to avoid side effect on failure
    auto ret = _stop();
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "[%s][stop()] Failed to stop node", this->get_name());
        return ret;
    }
    //! Stop the token-based timer and reset it
    if (m_impl->read_frame_token) {
        m_impl->read_frame_token->stop();
    }

    //! Set status code to STOPPED
    _set_status_code(NodeStatusCode::STOPPED);
    return 0;
}

int RedoxiVideoReaderBase::close()
{
    //! Ensure the node is in OPENED or STOPPED state
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::OPENED ||
                              m_status_code == NodeStatusCode::STOPPED,
                          "[{}][close()] Invalid node status for closing: {}", this->get_name(), m_status_code);

    // do subclass work first, to avoid side effect on failure
    auto ret = _close();
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "[%s][close()] Failed to close node", this->get_name());
        return ret;
    }

    //! Set status code to CLOSED
    _set_status_code(NodeStatusCode::CLOSED);

    return 0;
}

int RedoxiVideoReaderBase::get_status_code() const
{
    return m_status_code;
}


int RedoxiVideoReaderBase::_connect_to_downstreams()
{
    //! Ensure m_init_config is set before connecting to downstreams
    RDX_ASSERT_CHECK_TRUE(m_init_config != nullptr,
                          "[{}][_connect_to_downstreams()] Init config is not set", this->get_name());

    // find and connect to downstreams
    m_downstreams.clear();
    for (auto &it : m_init_config->downstreams) {
        auto ds = std::make_shared<Downstream_t>();

        // connect to accept_frame action server of downstream
        {
            std::string name = it.second->accept_frame_action;
            auto client = rclcpp_action::create_client<ACT_AcceptFrame_t>(this, name);
            ds->accept_frame = client;

            // wait until the action server becomes online
            RCLCPP_DEBUG(this->get_logger(), "[%s][_connect_to_downstreams()] Waiting for action server %s to be online", this->get_name(), name.c_str());
            client->wait_for_action_server();
            RCLCPP_DEBUG(this->get_logger(), "[%s][_connect_to_downstreams()] Action server %s connected", this->get_name(), name.c_str());
        }

        m_downstreams[it.first] = ds;
    }

    return 0;
}

int RedoxiVideoReaderBase::_init_frame_delivery_tasks()
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
    using FrameDeliveryNode_t = ap::SingleBufferExecNode<mytypes::FrameDeliveryTask, mytypes::FrameDeliveryTask>;
    using WorkInput_t = FrameDeliveryNode_t::InputWithTokens_t;
    using WorkOutput_t = FrameDeliveryNode_t::OutputWithTokens_t;

    // setup work function, nothing to do because during work function
    // frames are out of order
    node.set_work_function(
        [this](const WorkInput_t &input, WorkOutput_t &output) -> int {
            // copy input to output
            auto &out_payload = std::get<0>(output);
            const auto &in_payload = std::get<0>(input);
            auto ret = this->_do_frame_delivery_preprocess(in_payload, out_payload);
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "[%s][WorkFunc] Failed to preprocess frame, error code: %d", this->get_name(), ret);
            }

            return ret;
        });

    // output callback
    // send frame to downstreams
    node.set_output_callback(
        [this](const WorkOutput_t &output) -> int {
            auto &out_payload = std::get<0>(output);
            auto ret = this->_do_frame_delivery_main(out_payload);
            if (ret != 0) {
                RCLCPP_ERROR(this->get_logger(), "[%s][WorkFunc] Failed to deliver frame, error code: %d", this->get_name(), ret);
            }
            return ret;
        });

    return 0;
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

int RedoxiVideoReaderBase::_do_frame_delivery_preprocess(
    const FrameDeliveryTask_t &task_input,
    FrameDeliveryTask_t &task_output)
{
    //! Copy input to output
    task_output = task_input;

    // resize image
    auto target_image_size = this->m_runtime_config->output_image_size;
    if (target_image_size.width != -1 || target_image_size.height != -1) {
        cv::resize(task_input.frame, task_output.frame, target_image_size);
    }

    return 0;
}

void RedoxiVideoReaderBase::_create_frame_delivery_task(
    const cv::Mat &frame,
    FrameDeliveryTask_t &task_output)
{
    //! Create a unique identifier for this task
    task_output.create_uuid();

    //! Shallow copy the frame
    task_output.frame = frame;

    //! Set the frame number
    task_output.frame_number = ++m_frame_number;

    //! Set the timestamp
    task_output.timestamp_sec = this->now().seconds();
}

int RedoxiVideoReaderBase::_do_frame_delivery_main(const FrameDeliveryTask_t &task_input)
{
    // check if any downstream is ready to accept new frame
    bool downstream_ready = _check_if_any_downstream_is_ready();
    if (!downstream_ready) {
        RCLCPP_WARN(this->get_logger(), "[%s][_do_frame_delivery_main()] No downstream is ready to accept new frame", this->get_name());
        return 0;
    }

    // add frame to shared memory
    uint64_t shared_memory_id = 0;
    auto ret = _add_frame_to_shared_memory(task_input.frame, shared_memory_id);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "[%s][_do_frame_delivery_main()] Failed to add frame to shared memory", this->get_name());
        return -1;
    }

    // create a frame message
    FrameMessage_t frame_msg = _create_frame_message(task_input, shared_memory_id);

    // send frame to downstreams
    bool sent = _send_frame_to_downstreams(frame_msg);

    // clean up resources
    if (!sent) {
        // if not sent, remove the frame from shared memory
        RCLCPP_ERROR(this->get_logger(), "[%s][_do_frame_delivery_main()] Failed to send frame to downstreams", this->get_name());
        _remove_frame_from_shared_memory(shared_memory_id);
    } else {
        // if sent, we are done, the shared memory is managed by downstreams
        RCLCPP_INFO(this->get_logger(), "[%s][_do_frame_delivery_main()] Frame sent to downstreams", this->get_name());
    }

    return 0;
}

RedoxiVideoReaderBase::FrameMessage_t RedoxiVideoReaderBase::_create_frame_message(
    const FrameDeliveryTask_t &task_input,
    std::optional<uint64_t> shared_memory_id)
{
    FrameMessage_t frame_msg;
    frame_msg.frame_num = task_input.frame_number;
    if (shared_memory_id) {
        frame_msg.cache.id_int = shared_memory_id.value();
        frame_msg.cache.has_int_id = true;
        frame_msg.cache.id_string = vineyard::ObjectIDToString(shared_memory_id.value());
    }
    return frame_msg;
}

int RedoxiVideoReaderBase::_add_frame_to_shared_memory(
    const cv::Mat &frame, uint64_t &shared_memory_id)
{
    if (!m_impl->v6d_client || !m_impl->v6d_client->is_connected()) {
        RCLCPP_ERROR(this->get_logger(), "[%s][_add_frame_to_shared_memory()] Vineyard client is not initialized or not connected", this->get_name());
        return -1;
    }

    // add frame to vineyard shared memory
    auto ret = m_impl->v6d_client->write_cvmat(frame, shared_memory_id);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "[%s][_add_frame_to_shared_memory()] Failed to add frame to shared memory", this->get_name());
    }
    return ret;
}

int RedoxiVideoReaderBase::_remove_frame_from_shared_memory(
    uint64_t shared_memory_id)
{
    if (!m_impl->v6d_client || !m_impl->v6d_client->is_connected()) {
        RCLCPP_ERROR(this->get_logger(), "[%s][_remove_frame_from_shared_memory()] Vineyard client is not initialized or not connected", this->get_name());
        return -1;
    }

    auto ret = m_impl->v6d_client->delete_object(shared_memory_id);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "[%s][_remove_frame_from_shared_memory()] Failed to remove frame from shared memory", this->get_name());
    }
    return ret;
}

int RedoxiVideoReaderBase::_send_frame_to_downstream(const FrameMessage_t &frame_msg,
                                                     const std::shared_ptr<Downstream_t> &ds,
                                                     bool is_blocking,
                                                     std::chrono::milliseconds timeout_ms)
{
    auto &client = ds->accept_frame;

    // create goal
    auto goal = Downstream_t::Goal_t();
    goal.frame = frame_msg;

    if (is_blocking) {
        auto goal_handle = client->send_goal(goal, ds->accept_frame_options);
    } else {
        auto goal_handle = client->async_send_goal(frame_msg, ds->accept_frame_options);
    }

    return 0;
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
                RCLCPP_ERROR(this->get_logger(), "[%s][_step()] Failed to read frame, error code: %d", this->get_name(), ret);
            } else {
                // create a frame delivery task and deliver
                FrameDeliveryTask_t task;
                _create_frame_delivery_task(frame, task);
                auto ret = m_impl->frame_delivery_node->put_data(task);
                if (!ret) {
                    RCLCPP_ERROR(this->get_logger(), "[%s][_step()] Failed to deliver frame", this->get_name());
                }
            }
        }
    }
}

} // namespace redoxi_works
