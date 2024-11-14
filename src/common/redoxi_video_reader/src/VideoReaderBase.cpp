// #include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <redoxi_video_reader/base/VideoReaderBaseTypes.hpp>
#include <redoxi_video_reader/base/VideoReaderBase.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
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

    // create shm client
    auto shm_config = shared_memory::SharedMemoryFactory::get_shm_config_from_node(this);
    RDX_INFO_DEV(this, __func__, "Creating shm client, region key={}, service name={}",
                 shm_config.region_key, shm_config.service_name);
    auto shm_client = shared_memory::SharedMemoryFactory::create_client_by_config(shm_config);
    if (shm_client == nullptr) {
        RDX_INFO_DEV(this, __func__, "{}", "Failed to create shm client, not using shared memory");
    } else {
        RDX_INFO_DEV(this, __func__, "Created shm client, region key={}, service name={}",
                     shm_config.region_key, shm_config.service_name);
        m_shm_client = shm_client;
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

    //! destroy shm client
    m_shm_client.reset();

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

int RedoxiVideoReaderBase::_on_delivery_task_begin(TargetData_t &target_data,
                                                   const DeliveryRequest_t &request)
{
    // ping request, do nothing
    if (request.is_ping_request()) {
        return 0;
    }

    // send data to shm if shm client is initialized
    auto msg_uuid_str = boost::uuids::to_string(request.get_source_data().get_uuid());

    // shm client is not initialized, do nothing
    if (!m_shm_client) {
        return 0;
    }

    // shm client is not connected, do nothing
    if (!m_shm_client->is_connected()) {
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Shm client is not connected", msg_uuid_str);
        return 0;
    }

    // write data to shm
    const auto &img = request.get_source_data().get_image();
    if (img.empty()) {
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Image is empty, nothing to do", msg_uuid_str);
        return 0;
    }

    // send image to shm
    RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] Uploading image to shm", msg_uuid_str);
    shared_memory::ObjectIdentifier oid;
    auto datablock = m_shm_client->create_datablock();
    if (!datablock) {
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Failed to create datablock", msg_uuid_str);
        return -1;
    }
    datablock->from_cvmat(img);
    bool put_ok = m_shm_client->put_data(&oid, datablock.get(), nullptr) == 0;
    if (!put_ok) {
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Failed to put data to shm", msg_uuid_str);
        return -1;
    }

    // set target data
    auto &shm_token = target_data.get_goal().frame.shm_token;
    if (oid.id.has_value()) {
        shm_token.object_id = oid.id.value();
    }
    if (oid.key.has_value()) {
        shm_token.object_key = oid.key.value();
    }
    shm_token.service_name = m_shm_client->get_service_name();
    shm_token.region_key = m_shm_client->get_region_key();
    shm_token.object_size = img.total() * img.elemSize();
    return 0;
}

int RedoxiVideoReaderBase::_on_delivery_task_finish(TargetData_t &target_data,
                                                    const DeliveryRequest_t &request,
                                                    const DeliveryResult_t &result)
{
    if (result.result_code == DeliveryResultCode::Success) {
        // success, do nothing
        return 0;
    }

    if (!m_shm_client || !m_shm_client->is_connected()) {
        // shm client is not initialized or not connected, do nothing, no shared memory to remove
        return 0;
    }

    auto msg_uuid_str = boost::uuids::to_string(request.get_source_data().get_uuid());

    // failed, remove shm object, if any
    RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] Failed to send data to shm, removing object from shm", msg_uuid_str);
    auto &shm_token = target_data.get_goal().frame.shm_token;
    if (shm_token.object_size >= 0) {
        shared_memory::ObjectIdentifier oid;
        if (shm_token.object_id != 0) {
            oid.id = shm_token.object_id;
        }
        if (!shm_token.object_key.empty()) {
            oid.key = shm_token.object_key;
        }
        auto ret = m_shm_client->delete_object(oid);
        if (ret != 0) {
            RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Failed to delete object from shm", msg_uuid_str);
            return -1;
        }
        RDX_INFO_DEV(this, __func__, true, "[msg_uuid={}] Removed object from shm", msg_uuid_str);
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

    //! set callback on request enqueued to resize image if needed
    auto output_image_size = m_runtime_config->output_image_size;
    m_primary_output_port->set_callback_on_request_enqueued([output_image_size](DeliveryRequest_t &request) {
        // resize image if needed
        auto original_size = request.get_source_data().get_image().size();
        if ((output_image_size.width <= 0 && output_image_size.height <= 0) || output_image_size == original_size) {
            return;
        }

        auto image = request.get_source_data().get_image();
        cv::Mat resized_image;

        if (output_image_size.width > 0 && output_image_size.height > 0) {
            cv::resize(image, resized_image, output_image_size);
        } else if (output_image_size.width > 0) {
            int new_height = static_cast<int>(original_size.height * (static_cast<double>(output_image_size.width) / original_size.width));
            new_height = std::max(1, new_height);
            cv::resize(image, resized_image, cv::Size(output_image_size.width, new_height));
        } else if (output_image_size.height > 0) {
            int new_width = static_cast<int>(original_size.width * (static_cast<double>(output_image_size.height) / original_size.height));
            new_width = std::max(1, new_width);
            cv::resize(image, resized_image, cv::Size(new_width, output_image_size.height));
        }

        request.get_source_data().set_image(resized_image);
    });

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

    // register callbacks
    port->set_callback_on_deliver_task_begin([this](TargetData_t &target_data, const DeliveryTask_t &task) {
        return _on_delivery_task_begin(target_data, task.get_request());
    });
    port->set_callback_on_deliver_task_finish([this](TargetData_t &target_data, const DeliveryTask_t &task, const DeliveryResult_t &result) {
        return _on_delivery_task_finish(target_data, task.get_request(), result);
    });
    port->set_callback_on_deliver_to_downstream_finish(
        [this](TargetData_t &target_data,
               SendResult_t &result,
               const DeliveryRequest_t &request,
               const Downstream_t &ds) {
            return _on_deliver_to_downstream_finish(target_data, result, request, ds);
        });

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

        // this is used for logging
        auto msg_uuid = source_data.get_uuid();

        // get qos, controls how to retry and drop frames
        auto &qos = m_runtime_config->frame_enqueue_policy;
        auto max_attempts = qos.get_retry_policy().get_number_of_retry(true).value() + 1;
        auto interval_between_attempts = qos.get_retry_policy().get_wait_time_between_retry(true).value();
        auto drop_frame_strategy = qos.get_drop_strategy();

        // start pushing request to output port
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
