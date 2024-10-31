// #include <redoxi_video_reader/base/VideoReaderBase_v2.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseTypes_v2.hpp>
#include <redoxi_video_reader/base/VideoReaderBase_v2.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <cv_bridge/cv_bridge.hpp>

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

void RedoxiVideoReaderBase_v2::set_publish_to_debug_topic(bool enable)
{
    m_publish_to_debug_topic = enable;
}

bool RedoxiVideoReaderBase_v2::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int RedoxiVideoReaderBase_v2::init(std::shared_ptr<InitConfig_t> config,
                                   std::shared_ptr<RuntimeConfig_t> runtime_config)
{
    //! Check if already initialized
    if (m_status_code != NodeStatusCode::BEFORE_INIT) {
        RDX_RAISE_ERROR("[{}] Cannot init in status {}", __func__, m_status_code);
    }

    //! Check input parameters
    if (!config || !runtime_config) {
        RDX_RAISE_ERROR("[{}] Invalid input parameters", __func__);
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

int RedoxiVideoReaderBase_v2::update_init_config(std::shared_ptr<InitConfig_t> config)
{
    //! Check input parameters
    if (!config) {
        RDX_RAISE_ERROR("[{}] Invalid input parameters", __func__);
        return -1;
    }

    //! Can only update init config in CLOSED status
    if (m_status_code != NodeStatusCode::CLOSED) {
        RDX_RAISE_ERROR("[{}] Cannot update init config in status {}", __func__, m_status_code);
        return -1;
    }

    //! Initialize output ports
    auto primary_output_port = _create_primary_output_port();
    if (!primary_output_port) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port = primary_output_port;

    //! Store configurations
    m_init_config = config;

    return 0;
}

int RedoxiVideoReaderBase_v2::update_runtime_config(std::shared_ptr<RuntimeConfig_t> config)
{
    //! Check input parameters
    if (!config) {
        RDX_RAISE_ERROR("[{}] Invalid input parameters", __func__);
        return -1;
    }

    //! Can only update runtime config in CLOSED or STOPPED status
    if (m_status_code != NodeStatusCode::CLOSED && m_status_code != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[{}] Cannot update runtime config in status {}", __func__, m_status_code);
        return -1;
    }

    //! Store configurations
    m_runtime_config = config;

    return 0;
}

std::shared_ptr<RedoxiVideoReaderImpl_v2> RedoxiVideoReaderBase_v2::_create_impl()
{
    auto impl = std::make_shared<RedoxiVideoReaderImpl_v2>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

void RedoxiVideoReaderBase_v2::_set_status_code(int status_code)
{
    m_status_code = status_code;
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
        auto qos = m_runtime_config->frame_request_policy;

        // publish
        auto max_attempts = qos->get_retry_policy()->get_number_of_retry(true).value() + 1;
        auto interval_between_attempts = qos->get_retry_policy()->get_wait_time_between_retry(true).value();
        auto drop_frame_strategy = qos->get_drop_strategy();

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
        } else {
            // Try up to max attempts if dropping is allowed
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                if (m_primary_output_port->try_push_request(delivery_request)) {
                    success = true;
                    break;
                }
                // wait for next attempt
                std::this_thread::sleep_for(interval_between_attempts);
            }
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
