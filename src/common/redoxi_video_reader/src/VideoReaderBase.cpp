#include <thread>

#include <redoxi_common_cpp/async_processor/SingleBufferExecNode.hpp>
#include <redoxi_common_cpp/redoxi_v6d.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_public_msgs/msg/frame.hpp>

#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseImpl.hpp>

#include <cv_bridge/cv_bridge.hpp>

namespace redoxi_works
{

//! Global switch to control whether thread ID will be printed
static const bool PRINT_THREAD_ID = true;

bool RedoxiVideoReaderBase::get_publish_to_debug_topic() const
{
    if (m_runtime_config) {
        return m_runtime_config->publish_to_debug_topic;
    }
    return false;
}

void RedoxiVideoReaderBase::set_publish_to_debug_topic(bool enable)
{
    // Note that it will only publish if the downstreams are also set to use debug pub
    // otherwise, it will be ignored

    //! Only update the runtime config if it is set
    if (m_runtime_config) {
        m_runtime_config->publish_to_debug_topic = enable;
    }
}

RedoxiVideoReaderBase::RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
    // declare parameters, should be called in constructor before everything else
    _declare_all_parameters();
}

void RedoxiVideoReaderBase::_set_status_code(int status_code)
{
    m_status_code = status_code;
}

std::shared_ptr<RedoxiVideoReaderImpl> RedoxiVideoReaderBase::_create_impl(const std::shared_ptr<InitConfig_t> &,
                                                                           const std::shared_ptr<RuntimeConfig_t> &)
{
    return std::make_shared<RedoxiVideoReaderImpl>(this);
}

int RedoxiVideoReaderBase::init(const std::shared_ptr<InitConfig_t> &config,
                                const std::shared_ptr<RuntimeConfig_t> &runtime_config)
{
    // must be in BEFORE_INIT status
    //! Ensure node is in BEFORE_INIT status
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::BEFORE_INIT,
                          "[init()] Invalid node status for initialization: {}", m_status_code);

    // both config and runtime config must be set
    RDX_ASSERT_CHECK_TRUE(config != nullptr,
                          "[init()] Init config is not set");
    RDX_ASSERT_CHECK_TRUE(runtime_config != nullptr,
                          "[init()] Runtime config is not set");

    // create the implementation
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Creating implementation ...");
    m_impl = _create_impl(config, runtime_config);

    // init shared memory storage if it is supported
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Initializing shared memory storage ...");
    if (m_impl->is_shared_memory_supported()) {
        int ret = m_impl->init_shared_memory_storage();
        if (ret != 0) {
            RDX_LOG_WARN(this, __func__, PRINT_THREAD_ID, "Failed to initialize shared memory storage, continue without shared memory");
        }
    }

    // set init_config
    {
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Updating init config ...");
        auto ret = update_init_config(config);
        if (ret != 0) {
            RDX_RAISE_ERROR("[init()] Failed to update init config");
            return ret;
        }
    }

    // set runtime_config
    {
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Updating runtime config ...");
        auto ret = update_runtime_config(runtime_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("[init()] Failed to update runtime config");
            return ret;
        }
    }

    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "Initialization completed successfully");
    return 0;
}

int RedoxiVideoReaderBase::update_init_config(const std::shared_ptr<InitConfig_t> &config)
{
    //! Ensure the node is in CLOSED or BEFORE_INIT status
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::CLOSED ||
                              m_status_code == NodeStatusCode::BEFORE_INIT,
                          "[update_init_config()] Invalid node status for updating init config: {}",
                          m_status_code);

    //! Ensure the new config is not null
    RDX_ASSERT_CHECK_TRUE(config != nullptr,
                          "[update_init_config()] New init config is null");

    //! Update the init config
    m_init_config = config;

    //! Reconnect to downstreams with the new config
    auto ret = _connect_to_downstreams();
    if (ret != 0) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "Failed to connect to downstreams, error code: {}", ret);
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
                          "[{}()] Invalid node status for updating runtime config: {}",
                          __func__, m_status_code);

    //! Ensure the new config is not null
    RDX_ASSERT_CHECK_TRUE(config != nullptr,
                          "[{}()] New runtime config is null", __func__);

    //! Update the runtime config
    m_runtime_config = config;

    // create timer based token for reading a new frame every x-milliseconds
    {
        auto interval = m_runtime_config->get_frame_interval_as_std_chrono();
        m_impl->read_frame_token = std::make_shared<RosTimeToken>(this, interval);
    }

    //! create frame delivery tasks
    {
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Initializing frame delivery tasks ...");
        int ret = _init_frame_delivery_tasks();
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}()] Failed to initialize frame delivery tasks", __func__);
            return ret;
        }
    }

    //! Apply any necessary changes based on the new runtime config
    //! This might involve updating internal parameters or reconfiguring components

    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "Runtime config updated successfully");

    return 0;
}

int RedoxiVideoReaderBase::open()
{
    // already opened? do nothing
    if (m_status_code == NodeStatusCode::OPENED) {
        return 0;
    }

    //! Ensure the node is in CLOSED state
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::CLOSED,
                          "[open()] Invalid node status for opening: {}", m_status_code);

    // do subclass work first, to avoid side effect on failure
    auto ret = _open();
    if (ret != 0) {
        RDX_LOG_ERROR(this, __func__, PRINT_THREAD_ID, "Failed to open node");
        return ret;
    }

    // reset frame number
    m_frame_number = -1;

    //! Set status code to opened
    _set_status_code(NodeStatusCode::OPENED);

    return 0;
}

int RedoxiVideoReaderBase::start()
{
    // already started? do nothing
    if (m_status_code == NodeStatusCode::STARTED) {
        return 0;
    }

    //! Ensure the node is in OPENED state
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::OPENED ||
                              m_status_code == NodeStatusCode::STOPPED,
                          "[start()] Invalid node status for starting: {}", m_status_code);

    // do subclass work first, to avoid side effect on failure
    auto ret = _start();
    if (ret != 0) {
        RDX_LOG_ERROR(this, __func__, PRINT_THREAD_ID, "Failed to start node");
        return ret;
    }

    // create a timer based token for reading a new frame every x-milliseconds
    {
        auto interval = m_runtime_config->get_frame_interval_as_std_chrono();
        m_impl->read_frame_token->start(interval);
    }

    //! Set status code to STARTED
    _set_status_code(NodeStatusCode::STARTED);

    // record the time when the node is started
    m_impl->time_node_last_started = this->now();

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
    // already stopped? do nothing
    if (m_status_code == NodeStatusCode::STOPPED) {
        return 0;
    }

    //! Ensure the node is in STARTED state
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::STARTED,
                          "[stop()] Invalid node status for stopping: {}", m_status_code);

    // do subclass work first, to avoid side effect on failure
    auto ret = _stop();
    if (ret != 0) {
        RDX_LOG_ERROR(this, __func__, PRINT_THREAD_ID, "Failed to stop node");
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
    // already closed? do nothing
    if (m_status_code == NodeStatusCode::CLOSED) {
        return 0;
    }

    //! Ensure the node is in OPENED/STARTED/STOPPED state
    RDX_ASSERT_CHECK_TRUE(m_status_code == NodeStatusCode::OPENED ||
                              m_status_code == NodeStatusCode::STARTED ||
                              m_status_code == NodeStatusCode::STOPPED,
                          "[close()] Invalid node status for closing: {}", m_status_code);

    // if in started state, stop the node first
    if (m_status_code == NodeStatusCode::STARTED) {
        auto ret = stop();
        if (ret != 0) {
            RDX_LOG_ERROR(this, __func__, PRINT_THREAD_ID, "Failed to stop node");
            return ret;
        }
    }

    // do subclass work first, to avoid side effect on failure
    auto ret = _close();
    if (ret != 0) {
        RDX_LOG_ERROR(this, __func__, PRINT_THREAD_ID, "Failed to close node");
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
                          "[_connect_to_downstreams()] Init config is not set");

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
            RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Waiting for action server {} to be online", name);
            client->wait_for_action_server();
            RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Action server {} connected", name);
        }

        // create publisher for debug image topic
        if (ds->spec->use_debug_pub) {
            ds->debug_pub_sending.init(this, ds->get_debug_pub_sending_name(*it.second));
            ds->debug_pub_dropped.init(this, ds->get_debug_pub_dropped_name(*it.second));
            ds->debug_pub_sent.init(this, ds->get_debug_pub_sent_name(*it.second));
        }

        m_downstreams[it.first] = ds;
    }

    return 0;
}

int RedoxiVideoReaderBase::_init_frame_delivery_tasks()
{
    // create graph and node
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Creating graph ...");
    m_impl->frame_delivery_graph = std::make_shared<tbb::flow::graph>();
    auto &g = *m_impl->frame_delivery_graph;

    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Creating frame delivery node ...");
    m_impl->frame_delivery_node = std::make_shared<
        ap::SingleBufferExecNode<
            mytypes::FrameDeliveryTask,
            mytypes::FrameDeliveryTask>>(g);
    auto &node = *m_impl->frame_delivery_node;
    {
        auto is_built = node.is_built();
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Frame delivery node is built: {}", is_built ? "true" : "false");
    }

    // set node params
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Setting node params ...");
    auto buffer_size = m_runtime_config->frame_delivery_options->num_buffer_frames;
    node.set_input_data_buffer_size(buffer_size);
    node.set_preserve_order(true);

    // sync mode, all functions are executed in the graph
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Setting node to sync mode ...");
    node.set_use_async_callback(false);
    using FrameDeliveryNode_t = ap::SingleBufferExecNode<mytypes::FrameDeliveryTask, mytypes::FrameDeliveryTask>;
    using WorkInput_t = FrameDeliveryNode_t::InputWithTokens_t;
    using WorkOutput_t = FrameDeliveryNode_t::OutputWithTokens_t;

    // setup work function, nothing to do because during work function
    // frames are out of order
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Setting work function ...");
    node.set_work_function(
        [this](const WorkInput_t &input, WorkOutput_t &output) -> int {
            // copy input to output
            auto &out_payload = std::get<0>(output);
            const auto &in_payload = std::get<0>(input);
            auto ret = this->_do_frame_delivery_preprocess(in_payload, out_payload);
            if (ret != 0) {
                RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "Failed to preprocess frame, error code: {}", ret);
            }

            return ret;
        });

    // output callback
    // send frame to downstreams
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Setting output callback ...");
    node.set_output_callback(
        [this](const WorkOutput_t &output) -> int {
            auto &out_payload = std::get<0>(output);
            auto ret = this->_do_frame_delivery_main(out_payload);
            if (ret != 0) {
                RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "Failed to deliver frame, error code: {}", ret);
            }
            return ret;
        });

    // build the node
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Building frame delivery node ...");
    node.build();

    return 0;
}

RedoxiVideoReaderBase::~RedoxiVideoReaderBase()
{
    if (m_impl) {
        // has init, close it
        this->close();
    }
}

int RedoxiVideoReaderBase::_declare_all_parameters()
{
    auto ret = declare_default_parameters_for_node(this);
    return ret;
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
    task_output.frame_number = m_frame_number;

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
        RDX_LOG_WARN(this, __func__, PRINT_THREAD_ID, "No downstream is ready to accept new frame");
        return 0;
    }

    // add frame to shared memory
    auto payload_type = m_runtime_config->frame_delivery_options->frame_payload_type;

    // create a frame message
    FrameMessage_t frame_msg;
    auto func_deliver_frame = [this](const FrameMessage_t &frame_msg) -> int {
        int ret = 0;
        bool sent = false;
        for (auto &it : m_downstreams) {
            auto ds = it.second;
            auto ret = _deliver_frame(frame_msg, ds);
            if (ret != 0) {
                RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID,
                             "Failed to deliver frame to downstream {}",
                             ds->spec->accept_frame_action);
            } else {
                sent = true;
            }
        }
        if (!sent) {
            ret = -1;
        }
        return ret;
    };

    if (payload_type == FrameDeliveryOptions_t::FramePayloadType::UncompressedBySharedMemory) {

        //! Add frame to shared memory
        uint64_t shared_memory_id = 0;
        auto ret = m_impl->add_to_shared_memory(task_input.frame, shared_memory_id);
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}()] Failed to add frame to shared memory", __func__);
            return ret;
        }

        //! Create frame message with shared memory ID
        frame_msg = _create_frame_message(task_input, payload_type, shared_memory_id);

        //! Deliver frame
        auto ret_deliver_frame = func_deliver_frame(frame_msg);

        //! Remove frame from shared memory if failed to deliver
        if (ret_deliver_frame != 0) {
            //! Failed to deliver frame, removing frame from shared memory
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "Failed to deliver frame, removing frame from shared memory");
            m_impl->remove_from_shared_memory(shared_memory_id);
        }

        return ret_deliver_frame;
    } else {
        // create frame message using image content
        frame_msg = _create_frame_message(task_input, payload_type, std::nullopt);

        //! Deliver frame
        auto ret_deliver_frame = func_deliver_frame(frame_msg);
        if (ret_deliver_frame != 0) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "Failed to deliver frame");
        }

        return ret_deliver_frame;
    }
}

int RedoxiVideoReaderBase::_deliver_frame(
    const FrameMessage_t &frame_msg,
    const std::shared_ptr<Downstream_t> &ds)
{
    //! Prepare the goal
    // auto goal = Downstream_t::Goal_t();
    // goal.frame = frame_msg;

    //! Set up retry strategy
    int attempts = 0;

    //! +1 for the first attempt
    const int max_attempts = ds->spec->retry_strategy->get_max_number_of_retries() + 1;
    auto timeout_each_attempt = ds->spec->retry_strategy->get_wait_time_for_retry();
    auto msg_uuid = to_boost_uuid(frame_msg.uuid);

    //! Debug image message
    const sensor_msgs::msg::Image *debug_image_msg = nullptr;
    if (frame_msg.raw_image.data.size() > 0) {
        debug_image_msg = &frame_msg.raw_image;
    }
    bool publish_to_debug_topic = get_publish_to_debug_topic();

    while (attempts < max_attempts) {

        //! Publish the frame to the debug topic
        if (publish_to_debug_topic && debug_image_msg && ds->debug_pub_sending.valid()) {
            ds->debug_pub_sending.publish(*debug_image_msg,
                                          fmt::format("[SENDING] frame_num={}, attempts={}/{}", frame_msg.frame_num, attempts + 1, max_attempts),
                                          cv::Scalar(0, 0, 255));
        }

        //! Send the frame to the downstream
        auto result = _send_frame_to_downstream(frame_msg, ds, timeout_each_attempt);
        if (!result.goal_handle_future.valid()) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Not sending frame to downstream {}, goal handle future is invalid",
                         boost::uuids::to_string(msg_uuid), ds->spec->accept_frame_action);
        } else {
            bool wait_indefinitely = timeout_each_attempt < DefaultTimeUnit_t::zero();

            if (result.response_code) {
                // it has a response code, check it
                switch (*result.response_code) {
                    case ActionDownstreamResponse::ACCEPTED:
                        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Frame accepted by downstream {}",
                                     boost::uuids::to_string(msg_uuid), ds->spec->accept_frame_action);

                        //! Publish the frame sent message
                        if (publish_to_debug_topic && ds->debug_pub_sent.valid() && debug_image_msg) {
                            ds->debug_pub_sent.publish(*debug_image_msg,
                                                       fmt::format("[SENT] frame_num={}, attempts={}/{}", frame_msg.frame_num, attempts + 1, max_attempts),
                                                       cv::Scalar(0, 255, 0));
                        }
                        return 0; // Success
                    case ActionDownstreamResponse::REJECTED:
                        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Frame rejected by downstream {}",
                                     boost::uuids::to_string(msg_uuid), ds->spec->accept_frame_action);
                        break;
                    case ActionDownstreamResponse::TIMEOUT:
                        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Timeout while sending frame to downstream {}",
                                     boost::uuids::to_string(msg_uuid), ds->spec->accept_frame_action);
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
                    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Timeout while waiting for goal handle from downstream {}",
                                 boost::uuids::to_string(msg_uuid), ds->spec->accept_frame_action);
                }
            }
        }

        attempts++;
        if (attempts < max_attempts) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Retrying frame delivery to downstream {} (attempt {}/{})",
                         boost::uuids::to_string(msg_uuid), ds->spec->accept_frame_action, attempts + 1, max_attempts);
        }
    }

    if (publish_to_debug_topic && ds->debug_pub_dropped.valid() && debug_image_msg) {
        ds->debug_pub_dropped.publish(*debug_image_msg,
                                      fmt::format("[FAILED] frame_num={}, attempts={}/{}", frame_msg.frame_num, attempts + 1, max_attempts),
                                      cv::Scalar(255, 0, 0));
    }

    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID, "[msg_uuid={}] Failed to deliver frame to downstream {} after {} attempts",
                 boost::uuids::to_string(msg_uuid), ds->spec->accept_frame_action, max_attempts);
    return -1;
}


RedoxiVideoReaderBase::FrameMessage_t RedoxiVideoReaderBase::_create_frame_message(
    const FrameDeliveryTask_t &task_input,
    FrameDeliveryOptions_t::FramePayloadType payload_type,
    std::optional<uint64_t> shared_memory_id)
{
    FrameMessage_t frame_msg;
    frame_msg.uuid = to_ros_uuid_msg(task_input.uid);
    frame_msg.frame_num = task_input.frame_number;
    if (task_input.control_signal) {
        frame_msg.x_control = *task_input.control_signal;
    }
    auto source_image_encoding = m_runtime_config->output_image_encoding;
    if (payload_type == FrameDeliveryOptions_t::FramePayloadType::UncompressedBySharedMemory) {
        if (shared_memory_id) {
            frame_msg.cache.id_int = shared_memory_id.value();
            frame_msg.cache.has_int_id = true;
            frame_msg.cache.id_string = vineyard::ObjectIDToString(shared_memory_id.value());
        }
    } else if (payload_type == FrameDeliveryOptions_t::FramePayloadType::Uncompressed) {
        //! Convert cv::Mat to sensor_msgs::msg::Image using cv_bridge
        cv_bridge::CvImage(std_msgs::msg::Header(), source_image_encoding, task_input.frame)
            .toImageMsg(frame_msg.raw_image);
    } else if (payload_type == FrameDeliveryOptions_t::FramePayloadType::JpegEncoded ||
               payload_type == FrameDeliveryOptions_t::FramePayloadType::PngEncoded) {
        //! Lambda function to convert cv::Mat before encoding
        auto convert_frame = [&]() -> cv::Mat {
            if (source_image_encoding == "bgr8" || source_image_encoding == "bgra8" || source_image_encoding == "mono8" || source_image_encoding == "mono16") {
                return task_input.frame;
            } else if (source_image_encoding == "rgb8") {
                cv::Mat converted;
                cv::cvtColor(task_input.frame, converted, cv::COLOR_RGB2BGR);
                return converted;
            } else if (source_image_encoding == "rgba8") {
                cv::Mat converted;
                cv::cvtColor(task_input.frame, converted, cv::COLOR_RGBA2BGR);
                return converted;
            } else {
                RDX_RAISE_ERROR("[_create_frame_message()] Unsupported image encoding {} for compression", source_image_encoding);
                return cv::Mat(); // Return empty Mat on error
            }
        };

        if (payload_type == FrameDeliveryOptions_t::FramePayloadType::JpegEncoded) {
            auto quality = m_runtime_config->frame_delivery_options->jpeg_quality;
            cv::Mat frame_to_encode = convert_frame();

            //! Encode the image as JPEG without using cv_bridge
            std::vector<uchar> buffer;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
            cv::imencode(".jpg", frame_to_encode, buffer, params);

            //! Save the encoded image to frame_msg.encoded_image
            frame_msg.encoded_image.format = "jpeg";
            frame_msg.encoded_image.data = buffer;
        } else { // PngEncoded
            cv::Mat frame_to_encode = convert_frame();

            //! Encode the image as PNG without using cv_bridge
            std::vector<uchar> buffer;
            std::vector<int> params = {cv::IMWRITE_PNG_COMPRESSION, 9}; // PNG compression level 0-9
            cv::imencode(".png", frame_to_encode, buffer, params);

            //! Save the encoded image to frame_msg.encoded_image
            frame_msg.encoded_image.format = "png";
            frame_msg.encoded_image.data = buffer;
        }
    } else {
        RDX_RAISE_ERROR("[_create_frame_message()] Unsupported frame payload type: {}", (int)payload_type);
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
    goal.x_uid = frame_msg.uuid;

    //! Use SyncActionSender to send the goal and wait for the response
    SyncActionSender<Downstream_t::ActionType_t> sender(this);
    auto result = sender.send(goal, *client, timeout);

    return result;
}

bool RedoxiVideoReaderBase::_ping(const std::shared_ptr<Downstream_t> &ds,
                                  DefaultTimeUnit_t timeout)
{
    //! Create an empty frame message for pinging
    FrameMessage_t ping_msg;
    ping_msg.uuid = to_ros_uuid_msg(boost::uuids::random_generator()());
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

void RedoxiVideoReaderBase::_step()
{
    //! If not started yet, do nothing
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }

    //! Create a local task group
    //! Accumulate tasks and execute them in one go
    tbb::task_group unordered_tasks;

    //! Read next frame, if token is ready
    DummyTimeToken token;
    if (m_impl->read_frame_token->try_pop_token(token)) {
        //! Time to read a new frame
        cv::Mat frame;
        int ret = _read_frame(frame, m_frame_number);
        if (ret == 0) {
            //! Create a frame delivery task and deliver
            FrameDeliveryTask_t delivery_task;
            _create_frame_delivery_task(frame, delivery_task);

            auto task_func = [this, delivery_task]() {
                const auto &qos = m_runtime_config->frame_delivery_options;

                //! At least try once
                bool task_sent = m_impl->frame_delivery_node->put_data(delivery_task);

                //! If qos says you should retry until success, do so
                if (qos->drop_frame_strategy == FrameDeliveryOptions_t::DropFrameStrategy::NoDrop) {
                    auto max_attempts = DefaultParams::MaxNumberOfRetries;
                    int attempts = 0;
                    while (!task_sent && attempts < max_attempts) {
                        attempts++;
                        std::this_thread::sleep_for(qos->deliver_retry_interval);
                        task_sent = m_impl->frame_delivery_node->put_data(delivery_task);
                    }

                    //! Flush the graph, make sure the frame is delivered
                    m_impl->frame_delivery_graph->wait_for_all();
                }

                if (task_sent) {
                    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Frame {} (UID: {}) sent",
                                  (int)delivery_task.frame_number,
                                  delivery_task.get_uid_as_string());
                } else {
                    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID, "Frame {} (UID: {}) dropped",
                                  (int)delivery_task.frame_number,
                                  delivery_task.get_uid_as_string());
                }
            };

            //! Execute the task in isolation
            unordered_tasks.run(task_func);
        }
    }

    //! Wait for all tasks to complete
    unordered_tasks.wait();
} //! End of _step() function

} // namespace redoxi_works
