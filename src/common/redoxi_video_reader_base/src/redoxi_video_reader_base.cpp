#include <thread>

#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <redoxi_common_cpp/redoxi_v6d.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>

#include <redoxi_video_reader_base/redoxi_video_reader_base.hpp>
#include <redoxi_video_reader_base/redoxi_video_reader_types.hpp>
#include <redoxi_video_reader_base/redoxi_video_reader_base_impl.hpp>

namespace redoxi_works
{

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

    // init shared memory storage if it is supported
    if (m_impl->is_shared_memory_supported()) {
        int ret = m_impl->init_shared_memory_storage();
        if (ret != 0) {
            RCLCPP_WARN(this->get_logger(), "[%s][init()] Failed to initialize shared memory storage, continue without shared memory", this->get_name());
        }
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
        auto interval = m_runtime_config->get_frame_interval_as_std_chrono();
        m_impl->read_frame_token = std::make_shared<RosTimeToken>(this, interval);
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
        auto interval = m_runtime_config->get_frame_interval_as_std_chrono();
        m_impl->read_frame_token->start(interval);
    }

    //! Set status code to STARTED
    _set_status_code(NodeStatusCode::STARTED);

    // create a thread for step() function and do the job
    m_impl->step_thread = std::make_shared<std::thread>(
        [this]() {
            auto step_interval = m_runtime_config->get_step_interval_as_std_chrono();
            while (rclcpp::ok() && m_status_code == NodeStatusCode::STARTED) {
                this->_step();
                std::this_thread::sleep_for(step_interval);
            }

            // clean up before exiting
            m_impl->frame_delivery_graph->wait_for_all();
        });

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

    // stop the step thread
    if (m_impl->step_thread) {
        m_impl->step_thread->join();
        m_impl->step_thread.reset();
    }

    return 0;
}

int RedoxiVideoReaderBase::close()
{
    //! Ensure the node is in OPENED/STARTED/STOPPED state
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::OPENED ||
                              m_status_code == NodeStatusCode::STARTED ||
                              m_status_code == NodeStatusCode::STOPPED,
                          "[{}][close()] Invalid node status for closing: {}", this->get_name(), m_status_code);

    // if in started state, stop the node first
    if (m_status_code == NodeStatusCode::STARTED) {
        auto ret = stop();
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "[%s][close()] Failed to stop node", this->get_name());
            return ret;
        }
    }

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
            auto client = rclcpp_action::create_client<Downstream_t::ActionType_t>(this, name);
            ds->accept_frame = client;
            ds->spec = it.second;

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
    auto buffer_size = m_runtime_config->frame_delivery_options->num_buffer_frames;
    node.set_input_data_buffer_size(buffer_size);
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
    bool downstream_ready = false;
    for (auto &it : m_downstreams) {
        auto ds = it.second;
        downstream_ready = _ping(ds, DefaultParams::PingActionWaitTime);
        if (downstream_ready) {
            break;
        }
    }
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
    bool sent = false; // true if at least one downstream accepted the frame
    for (auto &it : m_downstreams) {
        auto ds = it.second;
        auto ret = _deliver_frame(frame_msg, ds);
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "[%s][_do_frame_delivery_main()] Failed to deliver frame to downstream %s",
                         this->get_name(), ds->spec->accept_frame_action.c_str());
        } else {
            sent = true;
        }
    }

    // clean up resources
    if (!sent) {
        // if not sent, remove the frame from shared memory
        RCLCPP_ERROR(this->get_logger(), "[%s][_do_frame_delivery_main()] Failed to send frame to downstreams", this->get_name());
        _remove_frame_from_shared_memory(shared_memory_id);
    }

    return 0;
}

int RedoxiVideoReaderBase::_deliver_frame(
    const FrameMessage_t &frame_msg,
    const std::shared_ptr<Downstream_t> &ds)
{
    //! Prepare the goal
    auto goal = Downstream_t::Goal_t();
    goal.frame = frame_msg;

    //! Set up retry strategy
    int attempts = 0;
    const int max_attempts = ds->spec->retry_strategy->get_max_number_of_retries();
    auto timeout_each_attempt = ds->spec->retry_strategy->get_wait_time_for_retry();

    while (attempts < max_attempts) {
        //! Send the frame to the downstream
        auto result = _send_frame_to_downstream(frame_msg, ds, timeout_each_attempt);
        bool wait_indefinitely = timeout_each_attempt < DefaultTimeUnit_t::zero();

        if (result.response_code) {
            // it has a response code, check it
            switch (*result.response_code) {
                case ActionDownstreamResponse::ACCEPTED:
                    return 0; // Success
                case ActionDownstreamResponse::REJECTED:
                    RCLCPP_WARN(this->get_logger(), "[%s][_deliver_frame()] Frame rejected by downstream %s",
                                this->get_name(), ds->spec->accept_frame_action.c_str());
                    break;
                case ActionDownstreamResponse::TIMEOUT:
                    RCLCPP_WARN(this->get_logger(), "[%s][_deliver_frame()] Timeout while sending frame to downstream %s",
                                this->get_name(), ds->spec->accept_frame_action.c_str());
                    break;
            }
        } else {
            // may or maynot have a response code, check the goal handle future
            if (wait_indefinitely) {
                //! Wait indefinitely for the goal handle future
                result.goal_handle_future.wait();
                auto goal_handle = result.goal_handle_future.get();
                if (goal_handle) {
                    return 0; // Success
                }
            } else {
                //! Regard as failure without additional waiting
                RCLCPP_WARN(this->get_logger(), "[%s][_deliver_frame()] Timeout while waiting for goal handle from downstream %s",
                            this->get_name(), ds->spec->accept_frame_action.c_str());
            }
        }

        attempts++;
        if (attempts < max_attempts) {
            RCLCPP_INFO(this->get_logger(), "[%s][_deliver_frame()] Retrying frame delivery to downstream %s (attempt %d/%d)",
                        this->get_name(), ds->spec->accept_frame_action.c_str(), attempts + 1, max_attempts);
        }
    }

    RCLCPP_ERROR(this->get_logger(), "[%s][_deliver_frame()] Failed to deliver frame to downstream %s after %d attempts",
                 this->get_name(), ds->spec->accept_frame_action.c_str(), max_attempts);
    return -1;
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
    return m_impl->add_to_shared_memory(frame, shared_memory_id);
}

int RedoxiVideoReaderBase::_remove_frame_from_shared_memory(
    uint64_t shared_memory_id)
{
    return m_impl->remove_from_shared_memory(shared_memory_id);
}

RedoxiVideoReaderBase::SendFrameResult_t
    RedoxiVideoReaderBase::_send_frame_to_downstream(
        const FrameMessage_t &frame_msg,
        const std::shared_ptr<Downstream_t> &ds,
        DefaultTimeUnit_t timeout)
{
    //! Get the action client for the downstream
    auto &client = ds->accept_frame;

    //! Create a goal object and populate it with frame message data
    auto goal = Downstream_t::Goal_t();
    goal.frame = frame_msg;
    goal.x_control = frame_msg.x_control;

    //! Use SyncActionSender to send the goal and wait for the response
    SyncActionSender<Downstream_t::ActionType_t> sender;
    auto result = sender.send(goal, *client, timeout);

    return result;
}

bool RedoxiVideoReaderBase::_ping(const std::shared_ptr<Downstream_t> &ds,
                                  DefaultTimeUnit_t timeout)
{
    //! Create an empty frame message for pinging
    FrameMessage_t ping_msg;
    ping_msg.x_control.code = ping_msg.x_control.PING;

    //! Send the ping message to the downstream
    auto result = _send_frame_to_downstream(ping_msg, ds, timeout);

    //! Check the response
    if (timeout == DefaultTimeUnit_t(0)) {
        //! If timeout is 0, return false as we cannot get a response in no time
        return false;
    } else if (timeout > DefaultTimeUnit_t(0)) {
        //! If timeout is positive, check the response code
        //! Anything other than ACCEPTED is considered not ready
        return result.response_code.has_value() &&
               result.response_code.value() == ActionDownstreamResponse::ACCEPTED;
    } else {
        //! If timeout is negative, wait indefinitely for the goal handle future
        result.goal_handle_future.wait();
        auto goal_handle = result.goal_handle_future.get();
        return goal_handle != nullptr;
    }
}

void RedoxiVideoReaderBase::_publish_frame(const cv::Mat &frame)
{
    //! Check if image publishing is enabled
    if (!m_publish_image || !m_topic_image) {
        return;
    }

    //! Create a sensor_msgs::msg::Image message
    auto image_msg = std::make_unique<sensor_msgs::msg::Image>();

    //! Set the image encoding based on the number of channels
    std::string encoding;
    switch (frame.channels()) {
        case 1:
            encoding = "mono8";
            break;
        case 3:
            encoding = "bgr8";
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported number of channels: %d", frame.channels());
            return;
    }

    //! Fill in the image message
    image_msg->header.stamp = this->now();
    image_msg->header.frame_id = "camera_frame";
    image_msg->height = frame.rows;
    image_msg->width = frame.cols;
    image_msg->encoding = encoding;
    image_msg->is_bigendian = false;
    image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    image_msg->data.assign(frame.datastart, frame.dataend);

    //! Publish the image message
    m_topic_image->publish(std::move(image_msg));
}


void RedoxiVideoReaderBase::_step()
{
    // if not started yet, do nothing
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }

    // create a local task group
    // accumulate tasks and execute them in one go
    tbb::task_group unordered_tasks;

    // read next frame, if token is ready
    DummyTimeToken token;
    if (m_impl->read_frame_token->try_pop_token(token)) {
        // time to read a new frame
        cv::Mat frame;
        int ret = _read_frame(frame);
        if (ret == 0) {
            // create a frame delivery task and deliver
            FrameDeliveryTask_t delivery_task;
            _create_frame_delivery_task(frame, delivery_task);

            auto task_func = [this, delivery_task]() {
                const auto &qos = m_runtime_config->frame_delivery_options;

                // at least try once
                bool task_sent = m_impl->frame_delivery_node->put_data(delivery_task);

                // if qos says you should retry until success, do so
                if (qos->drop_frame_strategy == FrameDeliveryOptions_t::DropFrameStrategy::NoDrop) {
                    auto max_attempts = DefaultParams::MaxNumberOfRetries;
                    int attempts = 0;
                    while (!task_sent && attempts < max_attempts) {
                        attempts++;
                        std::this_thread::sleep_for(qos->deliver_retry_interval);
                        task_sent = m_impl->frame_delivery_node->put_data(delivery_task);
                    }

                    // flush the graph, make sure the frame is delivered
                    m_impl->frame_delivery_graph->wait_for_all();
                }

                if (task_sent) {
                    RCLCPP_DEBUG(this->get_logger(), "[%s][_step()] Frame %d (UID: %s) sent",
                                 this->get_name(), (int)delivery_task.frame_number,
                                 delivery_task.get_uid_as_string().c_str());
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "[%s][_step()] Frame %d (UID: %s) dropped",
                                 this->get_name(), (int)delivery_task.frame_number,
                                 delivery_task.get_uid_as_string().c_str());
                }
            };

            // execute the task in isolation
            unordered_tasks.run(task_func);

            // requested publish frame?
            if (m_publish_image) {
                unordered_tasks.run([this, frame]() {
                    _publish_frame(frame);
                });
            }
        }
    }

    // wait for all tasks to complete
    unordered_tasks.wait();
} // end of _step() function

} // namespace redoxi_works
