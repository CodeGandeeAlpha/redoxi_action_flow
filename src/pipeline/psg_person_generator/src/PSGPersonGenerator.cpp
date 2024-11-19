#include <psg_person_generator/PSGPersonGeneratorTypes.hpp>
#include <psg_person_generator/PSGPersonGenerator.hpp>
#include <psg_common/msg_converter.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <json_struct/json_struct.h>

#define PRINT_THREAD_ID_IN_LOG (true)

namespace redoxi_works
{

struct PSGPersonGeneratorImpl {
    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    // PASSENGERFLOW person extractor
    PassengerFlow::PersonExtractor m_person_extractor;
};

PSGPersonGenerator::PSGPersonGenerator(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
    _declare_all_parameters();
}

PSGPersonGenerator::~PSGPersonGenerator()
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

int PSGPersonGenerator::start()
{
    //! Can only start in STOPPED status
    if (m_status_code != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[{}] status must be in STOPPED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

    //! Start input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting psg master node");
    m_input_port->start();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port started");

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

int PSGPersonGenerator::stop()
{
    //! Can only stop in STARTED status
    if (m_status_code != NodeStatusCode::STARTED) {
        RDX_RAISE_ERROR("[{}] status must be in STARTED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

    //! Stop input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Stopping psg master node");
    m_input_port->stop();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port stopped");


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

void PSGPersonGenerator::set_publish_to_debug_topic(bool enable)
{
    m_publish_to_debug_topic = enable;
    if (m_publish_to_debug_topic) {
        if (m_primary_output_port) {
            m_primary_output_port->set_publish_to_debug_topic(enable);
        }
    }
}

bool PSGPersonGenerator::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int PSGPersonGenerator::init(std::shared_ptr<InitConfig_t> config,
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

    // create the input port
    m_input_port = std::make_shared<InputPort_t>(this);
    m_input_port->init(m_init_config->input_port_config);

    //! Change status to STOPPED
    _set_status_code(NodeStatusCode::STOPPED);

    return 0;
}

int PSGPersonGenerator::update_init_config(std::shared_ptr<InitConfig_t> config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "update init config");
    //! Can only update init config in BEFORE_INIT or STOPPED status
    if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[{}] status must be in BEFORE_INIT or STOPPED, got {}", __func__, NodeStatusCodeToString(m_status_code));
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

int PSGPersonGenerator::update_runtime_config(std::shared_ptr<RuntimeConfig_t> config)
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
    m_primary_output_port->set_callback_on_request_enqueued([](DeliveryRequest_t &request) {
        // do nothing
    });

    //! set publish to debug topic
    set_publish_to_debug_topic(config->publish_to_debug_topic);

    return 0;
}

std::shared_ptr<PSGPersonGeneratorImpl> PSGPersonGenerator::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGPersonGeneratorImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

void PSGPersonGenerator::_set_status_code(int status_code)
{
    m_status_code = status_code;
}

PSGPersonGenerator::DeliveryRequest_t
    PSGPersonGenerator::_create_delivery_request(const OutputSourceData_t &source_data)
{
    //! Create delivery request
    DeliveryRequest_t req;
    req.set_source_data(source_data);
    if (m_runtime_config->frame_request_policy.has_value()) {
        req.set_delivery_policy(*m_runtime_config->frame_request_policy);
    }

    return req;
}

std::shared_ptr<PSGPersonGenerator::OutputPort_t>
    PSGPersonGenerator::_create_primary_output_port()
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port");
    auto port = std::make_shared<OutputPort_t>(this);
    auto &port_config = m_init_config->output_port_config;
    // RDX_ASSERT_CHECK_TRUE(!port_config.get_downstream_specs().empty(),
    //                       "[{}] port_config must have at least one downstream", __func__);
    port->init(port_config);

    // // register callbacks
    // port->set_callback_on_deliver_task_begin([this](TargetData_t &target_data, const DeliveryTask_t &task) {
    //     return _on_delivery_task_begin(target_data, task.get_request());
    // });
    // port->set_callback_on_deliver_task_finish([this](TargetData_t &target_data, const DeliveryTask_t &task, const DeliveryResult_t &result) {
    //     return _on_delivery_task_finish(target_data, task.get_request(), result);
    // });
    // port->set_callback_on_deliver_to_downstream_finish([this](TargetData_t &target_data, SendResult_t &result, const Downstream_t &ds) {
    //     return _on_deliver_to_downstream_finish(target_data, result, ds);
    // });

    return port;
}

int PSGPersonGenerator::_declare_all_parameters()
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

void PSGPersonGenerator::_step()
{
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }

    // get input document msg
    if (m_impl->m_ros_time_token->try_pop_token()) {
        std::shared_ptr<InputSourceData_t> document_in_data;
        if (m_init_config->enable_blocking_mode) {
            // wait until there is data available
            document_in_data = m_input_port->pop_source_data();
        } else {
            // try to get data without waiting
            document_in_data = m_input_port->try_pop_source_data();
        }

        if (!document_in_data) {
            return;
        }

        // process document, copy the document msg because the original one is const, cannot be modified
        psg_private_msgs::msg::PsgDocument document_msg;
        document_msg = document_in_data->m_goal->document;

        std::vector<PassengerFlow::DetectionPtr> v_detections;
        FlowRos2Pipeline::convert_msg_to_detections(document_msg.detections, v_detections);
        // extract person
        auto v_persons = m_impl->m_person_extractor.extract_persons(v_detections);

        // convert to msg
        psg_private_msgs::msg::Persons persons_msg;
        FlowRos2Pipeline::convert_persons_to_msg(v_persons, document_msg.frame, persons_msg);

        // create tasks
        document_msg.persons = persons_msg;

        // from input source data to output source data
        OutputSourceData_t output_source_data;
        output_source_data.set_document(document_msg);

        // create delivery request
        auto delivery_request = _create_delivery_request(output_source_data);

        // this is used for logging
        auto msg_uuid = output_source_data.get_uuid();

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
