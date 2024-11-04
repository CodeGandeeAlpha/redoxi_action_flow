// #include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <json_struct/json_struct.h>

#define PRINT_THREAD_ID_IN_LOG (true)

namespace redoxi_works
{

struct RedoxiVideoReaderImpl {
    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;
};

RedoxiVideoReaderBase::RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
    _declare_all_parameters();
}

RedoxiVideoReaderBase::~RedoxiVideoReaderBase()
{
    // wait for all requests to be processed
    if (m_primary_output_port) {
        m_primary_output_port->wait_for_all_requests();
    }

    // stop ros time token
    if (m_impl->m_ros_time_token) {
        m_impl->m_ros_time_token->stop();
    }

    // stop step thread
    m_step_running = false;
    if (m_step_thread != nullptr && m_step_thread->joinable()) {
        m_step_thread->join();
    }
}

int RedoxiVideoReaderBase::open()
{
    //! Can only open in CLOSED status
    if (m_status_code != NodeStatusCode::CLOSED) {
        RDX_RAISE_ERROR("[{}] status must be in CLOSED, got {}", __func__, m_status_code);
        return -1;
    }

    //! Reset frame number
    m_frame_number = -1;

    //! Call subclass open implementation
    auto ret = _open();
    if (ret != 0) {
        RDX_RAISE_ERROR("[{}] Failed to open video source, ret={}", __func__, ret);
        return ret;
    }

    //! Change status to OPENED
    _set_status_code(NodeStatusCode::OPENED);

    return 0;
}

int RedoxiVideoReaderBase::start()
{
    //! Can only start in OPENED or STOPPED status
    if (m_status_code != NodeStatusCode::OPENED && m_status_code != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[{}] status must be in OPENED or STOPPED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

    //! Start primary output port
    if (m_primary_output_port) {
        auto ret = m_primary_output_port->start();
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}] Failed to start primary output port, ret={}", __func__, ret);
            return ret;
        }
    }

    //! start ros time token
    {
        auto interval = m_runtime_config->frame_interval;
        m_impl->m_ros_time_token->start(interval);
    }

    //! Call subclass start implementation
    auto ret = _start();
    if (ret != 0) {
        RDX_RAISE_ERROR("[{}] Failed to start video source, ret={}", __func__, ret);
        return ret;
    }

    //! Change status to STARTED
    _set_status_code(NodeStatusCode::STARTED);

    //! Start step thread
    auto step_interval = m_runtime_config->step_interval;
    m_step_running = true;
    m_step_thread = std::make_shared<std::thread>([this, step_interval]() {
        while (m_status_code == NodeStatusCode::STARTED && rclcpp::ok() && m_step_running) {
            _step();
            std::this_thread::sleep_for(step_interval);
        }
    });

    return 0;
}

int RedoxiVideoReaderBase::stop()
{
    //! Can only stop in STARTED status
    if (m_status_code != NodeStatusCode::STARTED) {
        RDX_RAISE_ERROR("[{}] status must be in STARTED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

    //! Stop primary output port
    if (m_primary_output_port) {
        m_primary_output_port->stop();
    }

    //! stop ros time token
    m_impl->m_ros_time_token->stop();

    //! Call subclass stop implementation
    auto ret = _stop();
    if (ret != 0) {
        RDX_RAISE_ERROR("[{}] Failed to stop video source, ret={}", __func__, ret);
        return ret;
    }

    //! Change status to STOPPED
    _set_status_code(NodeStatusCode::STOPPED);

    //! Stop step thread
    m_step_running = false;
    if (m_step_thread != nullptr && m_step_thread->joinable()) {
        m_step_thread->join();
    }

    return 0;
}

int RedoxiVideoReaderBase::close()
{
    //! Can only close in OPENED or STOPPED status
    if (m_status_code != NodeStatusCode::OPENED && m_status_code != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[{}] status must be in OPENED or STOPPED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

    //! Call subclass close implementation
    auto ret = _close();
    if (ret != 0) {
        RDX_RAISE_ERROR("[{}] Failed to close video source, ret={}", __func__, ret);
        return ret;
    }

    //! Change status to CLOSED
    _set_status_code(NodeStatusCode::CLOSED);

    return 0;
}


void RedoxiVideoReaderBase::set_publish_to_debug_topic(bool enable)
{
    m_publish_to_debug_topic = enable;
    if (m_publish_to_debug_topic) {
        if (m_primary_output_port) {
            m_primary_output_port->set_publish_to_debug_topic(enable);
        }
    }
}

bool RedoxiVideoReaderBase::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int RedoxiVideoReaderBase::init(std::shared_ptr<InitConfig_t> config,
                                std::shared_ptr<RuntimeConfig_t> runtime_config)
{
    //! Check if already initialized
    if (m_status_code != NodeStatusCode::BEFORE_INIT) {
        RDX_RAISE_ERROR("[{}] status must be in BEFORE_INIT, got {}", __func__, NodeStatusCodeToString(m_status_code));
    }

    //! Create implementation details of this node
    //! @note this must be called before any other operations
    m_impl = _create_impl();

    //! apply or update init config
    update_init_config(config);

    //! apply or update runtime config
    update_runtime_config(runtime_config);

    //! Change status to CLOSED
    _set_status_code(NodeStatusCode::CLOSED);

    return 0;
}

int RedoxiVideoReaderBase::update_init_config(std::shared_ptr<InitConfig_t> config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "update init config");
    //! Can only update init config in BEFORE_INIT or CLOSED status
    if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
        RDX_RAISE_ERROR("[{}] status must be in BEFORE_INIT or CLOSED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

    //! Store configurations, this must come before other operations
    m_init_config = config;

    // parse the config into a string and print it
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "parse init config into a string");
    auto config_str = JS::serializeStruct(*config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "init config: {}", config_str);

    //! config must have some downstream
    // RDX_ASSERT_CHECK_TRUE(!config->primary_output_spec.get_downstream_specs().empty(),
    //                       "[{}] init config must have at least one downstream", __func__);

    //! Initialize output ports
    auto primary_output_port = _create_primary_output_port();
    if (!primary_output_port) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port = primary_output_port;

    //! Initialize debug publishers
    if (m_init_config->create_debug_pub) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "initialize debug publishers, enqueue topic={}, drop topic={}",
                     m_init_config->debug_pub_task_enqueue_name,
                     m_init_config->debug_pub_task_drop_name);
        auto debug_qos = DefaultParams::DebugPublisherQoS;
        m_pub_task_enqueue.init(this, m_init_config->debug_pub_task_enqueue_name, debug_qos);
        m_pub_task_drop.init(this, m_init_config->debug_pub_task_drop_name, debug_qos);
    }

    return 0;
}

int RedoxiVideoReaderBase::update_runtime_config(std::shared_ptr<RuntimeConfig_t> config)
{
    //! cannot be updated in STARTED status
    if (m_status_code == NodeStatusCode::STARTED) {
        RDX_RAISE_ERROR("[{}] status must not be in STARTED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

    //! parse the config into a string and print it
    auto config_str = JS::serializeStruct(*config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "runtime config: {}", config_str);

    //! Store configurations
    m_runtime_config = config;

    //! set publish to debug topic
    set_publish_to_debug_topic(config->publish_to_debug_topic);

    return 0;
}

std::shared_ptr<RedoxiVideoReaderImpl> RedoxiVideoReaderBase::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<RedoxiVideoReaderImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

void RedoxiVideoReaderBase::_set_status_code(int status_code)
{
    m_status_code = status_code;
}

RedoxiVideoReaderBase::DeliveryRequest_t
    RedoxiVideoReaderBase::_create_delivery_request(const SourceData_t &source_data)
{
    //! Create delivery request
    DeliveryRequest_t req;
    req.set_source_data(source_data);
    if (m_runtime_config->frame_request_policy.has_value()) {
        req.set_delivery_policy(*m_runtime_config->frame_request_policy);
    }

    return req;
}

std::shared_ptr<RedoxiVideoReaderBase::OutputPort_t>
    RedoxiVideoReaderBase::_create_primary_output_port()
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port");
    auto port = std::make_shared<OutputPort_t>(this);
    auto &port_config = m_init_config->primary_output_spec;
    // RDX_ASSERT_CHECK_TRUE(!port_config.get_downstream_specs().empty(),
    //                       "[{}] port_config must have at least one downstream", __func__);
    port->init(port_config);

    return port;
}

int RedoxiVideoReaderBase::_declare_all_parameters()
{
    auto ret = declare_default_parameters_for_node(this);
    if (ret != 0) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "Failed to declare default parameters for node, ret={}", ret);
        return ret;
    }

    // parse json parameters
    auto node = this;
    m_json_parameters = RDX_GET_JSON_PARAM_FROM_NODE(node);

    return 0;
}

void RedoxiVideoReaderBase::_step()
{
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }

    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "step once");

    if (m_impl->m_ros_time_token->try_pop_token()) {
        // time to get a new frame
        SourceData_t source_data;
        int ret = _read_frame(source_data, m_frame_number);
        if (ret != 0) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "Failed to read frame, ret={}", ret);
        }

        // create delivery request
        auto delivery_request = _create_delivery_request(source_data);
        auto msg_uuid = source_data.get_uuid();

        // get qos
        auto &qos = m_runtime_config->frame_enqueue_policy;

        // publish
        auto max_attempts = qos.get_retry_policy().get_number_of_retry(true).value() + 1;
        auto interval_between_attempts = qos.get_retry_policy().get_wait_time_between_retry(true).value();
        auto drop_frame_strategy = qos.get_drop_strategy();

        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "try to push request in {} attempts, retry interval={}ms",
                     max_attempts, interval_between_attempts.count());

        bool success = false;
        if (drop_frame_strategy == DropStrategy::NoDrop) {
            // Keep trying until success if no drop strategy
            while (!m_primary_output_port->try_push_request(delivery_request)) {
                std::this_thread::sleep_for(interval_between_attempts);
            }
            success = true;
        } else if (drop_frame_strategy == DropStrategy::DropAsNeeded) {
            // Try up to max attempts if dropping is allowed
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                if (m_primary_output_port->try_push_request(delivery_request)) {
                    success = true;
                    break;
                }
                // wait for next attempt
                std::this_thread::sleep_for(interval_between_attempts);
            }
        } else {
            RDX_RAISE_ERROR("[{}] invalid drop strategy, got {}", __func__, int(drop_frame_strategy));
        }

        if (success) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "[msg_uuid={}] success to push request",
                         boost::uuids::to_string(msg_uuid));
        } else {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "[msg_uuid={}] failed to push request",
                         boost::uuids::to_string(msg_uuid));
        }

        // FIXME: debug only
        // wait for all requests to be processed, not necessary
        m_primary_output_port->wait_for_all_requests();
    }
}

} // namespace redoxi_works
