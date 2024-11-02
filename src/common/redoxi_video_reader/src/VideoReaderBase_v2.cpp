// #include <redoxi_video_reader/base/VideoReaderBase_v2.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseTypes_v2.hpp>
#include <redoxi_video_reader/base/VideoReaderBase_v2.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <json_struct/json_struct.h>

#define PRINT_THREAD_ID_IN_LOG (true)

namespace v2 = redoxi_works::video_reader_base_v2;

namespace redoxi_works
{

struct RedoxiVideoReaderImpl_v2 {
    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;
};

RedoxiVideoReaderBase_v2::RedoxiVideoReaderBase_v2(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
    _declare_all_parameters();
}

RedoxiVideoReaderBase_v2::~RedoxiVideoReaderBase_v2()
{
    // wait for all requests to be processed
    if (m_primary_output_port) {
        m_primary_output_port->wait_for_all_requests();
    }
}

int RedoxiVideoReaderBase_v2::open()
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

int RedoxiVideoReaderBase_v2::start()
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

    //! Call subclass start implementation
    auto ret = _start();
    if (ret != 0) {
        RDX_RAISE_ERROR("[{}] Failed to start video source, ret={}", __func__, ret);
        return ret;
    }

    //! Change status to STARTED
    _set_status_code(NodeStatusCode::STARTED);

    return 0;
}

int RedoxiVideoReaderBase_v2::stop()
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

    //! Call subclass stop implementation
    auto ret = _stop();
    if (ret != 0) {
        RDX_RAISE_ERROR("[{}] Failed to stop video source, ret={}", __func__, ret);
        return ret;
    }

    //! Change status to STOPPED
    _set_status_code(NodeStatusCode::STOPPED);

    return 0;
}

int RedoxiVideoReaderBase_v2::close()
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


void RedoxiVideoReaderBase_v2::set_publish_to_debug_topic(bool enable)
{
    m_publish_to_debug_topic = enable;
}

bool RedoxiVideoReaderBase_v2::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int RedoxiVideoReaderBase_v2::init(const InitConfig_t &config,
                                   const RuntimeConfig_t &runtime_config)
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

int RedoxiVideoReaderBase_v2::update_init_config(const InitConfig_t &config)
{
    //! Can only update init config in BEFORE_INIT or CLOSED status
    if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::CLOSED) {
        RDX_RAISE_ERROR("[{}] status must be in BEFORE_INIT or CLOSED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

    // parse the config into a string and print it
    auto config_str = JS::serializeStruct(config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "init config: {}", config_str);

    //! Initialize output ports
    auto primary_output_port = _create_primary_output_port();
    if (!primary_output_port) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port = primary_output_port;

    //! Initialize debug publishers
    if (m_init_config.create_debug_pub) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "initialize debug publishers, enqueue topic={}, drop topic={}",
                     m_init_config.debug_pub_task_enqueue_name,
                     m_init_config.debug_pub_task_drop_name);
        auto debug_qos = DefaultParams::DebugPublisherQoS;
        m_pub_task_enqueue.init(this, m_init_config.debug_pub_task_enqueue_name, debug_qos);
        m_pub_task_drop.init(this, m_init_config.debug_pub_task_drop_name, debug_qos);
    }

    //! Store configurations
    m_init_config = config;

    return 0;
}

int RedoxiVideoReaderBase_v2::update_runtime_config(const RuntimeConfig_t &config)
{
    //! cannot be updated in STARTED status
    if (m_status_code == NodeStatusCode::STARTED) {
        RDX_RAISE_ERROR("[{}] status must not be in STARTED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

    //! parse the config into a string and print it
    auto config_str = JS::serializeStruct(config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "runtime config: {}", config_str);

    //! Store configurations
    m_runtime_config = config;

    return 0;
}

std::shared_ptr<RedoxiVideoReaderImpl_v2> RedoxiVideoReaderBase_v2::_create_impl()
{
    auto impl = std::make_shared<RedoxiVideoReaderImpl_v2>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this, m_runtime_config.frame_interval);
    return impl;
}

void RedoxiVideoReaderBase_v2::_set_status_code(int status_code)
{
    m_status_code = status_code;
}

RedoxiVideoReaderBase_v2::DeliveryRequest_t
    RedoxiVideoReaderBase_v2::_create_delivery_request(const SourceData_t &source_data)
{
    //! Create delivery request
    DeliveryRequest_t req;
    req.set_source_data(source_data);
    req.set_delivery_policy(m_runtime_config.fallback_primary_output_policy);

    return req;
}

std::shared_ptr<RedoxiVideoReaderBase_v2::OutputPort_t>
    RedoxiVideoReaderBase_v2::_create_primary_output_port()
{
    auto port = std::make_shared<OutputPort_t>();
    // OutputPort_t::InitConfig_t port_config;
    // std::vector<OutputPort_t::DownstreamSpec_t> downstreams;
    // for (const auto &[node_name, ds] : m_init_config.downstreams) {
    //     downstreams.push_back(ds);
    // }
    // port_config.set_downstream_specs(downstreams);
    // port_config.set_num_buffer_requests(1); // always keep the latest one
    // port_config.set_preserve_request_order(true);
    auto &port_config = m_init_config.primary_output_spec;
    port->init(port_config);
    return port;
}

int RedoxiVideoReaderBase_v2::_declare_all_parameters()
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

void RedoxiVideoReaderBase_v2::_step()
{
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }

    if (m_impl->m_ros_time_token->try_pop_token()) {
        // time to get a new frame
        auto source_data = std::make_shared<SourceData_t>();
        int ret = _read_frame(*source_data, m_frame_number);
        if (ret != 0) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "Failed to read frame, ret={}", ret);
        }

        // create delivery request
        auto delivery_request = _create_delivery_request(*source_data);
        auto msg_uuid = source_data->get_uuid();

        // get qos
        auto &qos = m_runtime_config.frame_request_policy;

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
