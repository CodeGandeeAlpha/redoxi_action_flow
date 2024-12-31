#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <json_struct/json_struct.h>
#include <tbb/task_group.h>

#include <redoxi_video_reader/base/v2/VideoReaderBaseTypes.hpp>
#include <redoxi_video_reader/base/v2/VideoReaderBase.hpp>
#define PRINT_THREAD_ID_IN_LOG (true)

namespace redoxi_works::video_readers::v2
{

struct RedoxiVideoReaderImpl {
    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;
    tbb::task_group m_tasks;
};

RedoxiVideoReaderBase::RedoxiVideoReaderBase(const std::string &name, const rclcpp::NodeOptions &options)
    : BaseNode_t(name, options)
{
}

RedoxiVideoReaderBase::~RedoxiVideoReaderBase() noexcept
{
    // wait for all requests to be processed
    if (m_primary_output_port) {
        m_primary_output_port->wait_for_all_requests();
    }

    // stop ros time token
    if (m_impl->m_ros_time_token) {
        m_impl->m_ros_time_token->stop();
    }

    // step thread will be handled by base class
}

int RedoxiVideoReaderBase::_open()
{
    //! Reset frame number
    _reset_frame_number();

    return 0;
}

int RedoxiVideoReaderBase::_start()
{
    auto config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);

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
        auto interval = config->frame_interval;
        m_impl->m_ros_time_token->start(interval);
    }

    // state transition is handled by base class
    // step thread will be started by base class

    return 0;
}

int RedoxiVideoReaderBase::_stop()
{
    //! Stop primary output port
    if (m_primary_output_port) {
        m_primary_output_port->stop();
    }

    //! stop ros time token
    m_impl->m_ros_time_token->stop();

    // state transition is handled by base class
    // step thread will be stopped by base class

    return 0;
}

int RedoxiVideoReaderBase::_close()
{
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

int64_t RedoxiVideoReaderBase::get_last_read_frame_number() const
{
    return m_last_read_frame_number;
}

int RedoxiVideoReaderBase::_update_init_config(std::shared_ptr<RootInitConfig_t> config)
{
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(config);

    // parse the config into a string and print it
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "parse init config into a string");
    auto config_str = JS::serializeStruct(*init_config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "init config: {}", config_str);

    // create impl
    m_impl = _create_impl();

    //! Initialize output ports
    auto primary_output_port = _create_primary_output_port(*init_config);
    if (!primary_output_port) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port = primary_output_port;

    //! Initialize debug publishers
    if (init_config->create_debug_pub) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "initialize debug publishers, enqueue topic={}, drop topic={}",
                     init_config->debug_pub_task_enqueue_name,
                     init_config->debug_pub_task_drop_name);
        auto debug_qos = DefaultParams::DebugPublisherQoS;
        m_pub_task_enqueue.init(this, init_config->debug_pub_task_enqueue_name, debug_qos);
        m_pub_task_drop.init(this, init_config->debug_pub_task_drop_name, debug_qos);
    }

    return 0;
}

int RedoxiVideoReaderBase::_on_delivery_task_begin(TargetData_t &target_data,
                                                   const DeliveryRequest_t &request)
{
    // do nothing
    (void)target_data;
    (void)request;
    return 0;
}

int RedoxiVideoReaderBase::_on_delivery_task_finish(TargetData_t &target_data,
                                                    const DeliveryRequest_t &request,
                                                    const DeliveryResult_t &result)
{
    (void)target_data;
    (void)request;
    (void)result;
    return 0;
}

int RedoxiVideoReaderBase::_update_runtime_config(std::shared_ptr<RootRuntimeConfig_t> config)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(config);

    //! parse the config into a string and print it
    auto config_str = JS::serializeStruct(*runtime_config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "runtime config: {}", config_str);

    //! set callback on request enqueued to resize image if needed
    auto output_image_size = runtime_config->output_image_size;
    m_primary_output_port->set_callback_on_request_enqueued([this, output_image_size](DeliveryRequest_t &request) {
        (void)this;
        // resize image if needed
        auto original_width = request.get_source_data().get_primary_frame().get_metadata().width;
        auto original_height = request.get_source_data().get_primary_frame().get_metadata().height;
        if ((output_image_size.width <= 0 && output_image_size.height <= 0) || output_image_size == cv::Size(original_width, original_height)) {
            return;
        }

        auto fm = request.get_source_data().get_primary_frame().to_frame_mediator();
        auto image = fm.to_cv_image_shared();

        // empty image? skip processing
        if (image.empty()) {
            return;
        }

        cv::Mat resized_image;
        if (output_image_size.width > 0 && output_image_size.height > 0) {
            cv::resize(image, resized_image, output_image_size);
        } else if (output_image_size.width > 0) {
            int new_height = static_cast<int>(original_height * (static_cast<double>(output_image_size.width) / original_width));
            new_height = std::max(1, new_height);
            cv::resize(image, resized_image, cv::Size(output_image_size.width, new_height));
        } else if (output_image_size.height > 0) {
            int new_width = static_cast<int>(original_width * (static_cast<double>(output_image_size.height) / original_height));
            new_width = std::max(1, new_width);
            cv::resize(image, resized_image, cv::Size(new_width, output_image_size.height));
        }

        SourceData_t::FrameData_t frame_data;
        frame_data.from_raw_data({.image = resized_image, .metadata = fm.get_metadata()});

        // update size info
        // frame_data.metadata.width = resized_image.cols;
        // frame_data.metadata.height = resized_image.rows;
        image_utils::FrameMediator::make_metadata_compatible(&frame_data.get_metadata(), resized_image);

        // RDX_INFO_DEV(this, __func__, "after resized, frame number={}, width={}, height={}",
        //              frame_data.metadata.frame_num,
        //              frame_data.metadata.width,
        //              frame_data.metadata.height);
        request.get_source_data().set_primary_frame(frame_data);
    });

    //! set publish to debug topic
    set_publish_to_debug_topic(runtime_config->publish_to_debug_topic);

    return 0;
}

std::shared_ptr<RedoxiVideoReaderImpl> RedoxiVideoReaderBase::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<RedoxiVideoReaderImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

RedoxiVideoReaderBase::DeliveryRequest_t
    RedoxiVideoReaderBase::_create_delivery_request(const SourceData_t &source_data,
                                                    std::optional<ControlSignalCode> control_signal_code)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);

    //! Create delivery request
    DeliveryRequest_t req;
    req.set_source_data(source_data);
    if (runtime_config->frame_request_policy.has_value()) {
        req.set_delivery_policy(*runtime_config->frame_request_policy);
    }
    if (control_signal_code.has_value()) {
        req.set_control_signal_code(control_signal_code.value());
    }
    return req;
}

std::shared_ptr<RedoxiVideoReaderBase::OutputPort_t>
    RedoxiVideoReaderBase::_create_primary_output_port(const InitConfig_t &init_config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port");
    auto port = std::make_shared<OutputPort_t>(this);
    auto &port_config = init_config.primary_output_spec;
    // RDX_ASSERT_CHECK_TRUE(!port_config.get_downstream_specs().empty(),
    //                       "[{}] port_config must have at least one downstream", __func__);

    {
        auto num_downstreams = port_config->get_downstream_specs().size();
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "number of downstreams={}", num_downstreams);
        if (num_downstreams == 0) {
            RDX_WARN_PRODUCTION(this, __func__, "{}", "no downstream, the frame will not be sent to anywhere");
        }
    }
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

void RedoxiVideoReaderBase::_step()
{
    if (get_status() != NodeStatusCode::STARTED) {
        return;
    }

    if (m_impl->m_ros_time_token->try_pop_token()) {
        auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);

        // time to get a new frame
        SourceData_t source_data;
        auto ret_read_frame = _read_frame(source_data, m_last_read_frame_number);
        switch (ret_read_frame) {
            case ReadFrameResult::OK: // good, pass it to downstream
                break;
            case ReadFrameResult::END_OF_VIDEO: // end of video, send this frame and then stop
                RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "End of video, send this frame and then stop");
                // the source data will still be sent with its original content, but it will be marked as a flush signal
                break;
            case ReadFrameResult::NO_DATA: // no data, try again
                return;
            case ReadFrameResult::ERROR: // unknown error, raise an error
                RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "Failed to read frame for unknown reason, skipping");
                return;
        }

        std::optional<ControlSignalCode> control_signal_code = std::nullopt;
        if (ret_read_frame == ReadFrameResult::END_OF_VIDEO) {
            // end of video, flush the pipeline
            control_signal_code = ControlSignalCode::Flush;
        }

        // create delivery request
        auto delivery_request = _create_delivery_request(source_data, control_signal_code);

        // this is used for logging
        auto msg_uuid = source_data.get_uuid();

        // get qos, controls how to retry and drop frames
        auto qos = runtime_config->frame_enqueue_policy;
        if (control_signal_code == ControlSignalCode::Flush || control_signal_code == ControlSignalCode::Terminate) {
            // end of video, flush or terminate signal must be sent reliably
            RDX_INFO_DEV(this, __func__, "{}", "end of video, flush or terminate signal must be sent reliably");
            qos.set_precondition(DeliveryPrecondition::NoPrecondition);
            qos.set_drop_strategy(DropStrategy::NoDrop);
            delivery_request.make_reliable();
        }

        // call callback before enqueue
        bool user_reject = false;
        {
            auto ret = _on_before_request_enqueue(delivery_request, qos);
            if (ret != 0) {
                RDX_INFO_DEV(this, __func__, "[msg_uuid={}] User rejected the request, error code={}",
                             boost::uuids::to_string(msg_uuid), ret);
                user_reject = true;
            }
        }

        // push request to output port
        bool success = false;
        if (!user_reject) {
            auto fm = source_data.get_primary_frame().to_frame_mediator();
            auto frame_number = fm.get_frame_number();
            auto source_frame_index = fm.get_source_frame_index();
            auto source_frame_timestamp = fm.get_source_timestamp_flat();
            auto task_metadata = delivery_request.get_source_task_metadata();
            auto task_uuid = task_metadata.source_task_id;
            RDX_INFO_DEV(this, __func__, "[msg_uuid={}][task={}] sending frame_number={}, source_frame_index={}, source_frame_timestamp={}, signal code={}, precondition={}, drop strategy={}",
                         boost::uuids::to_string(msg_uuid), UUIDTrait::to_string(task_uuid), frame_number, source_frame_index,
                         fmt::format("{:06f} sec", source_frame_timestamp.count() / 1e6),
                         control_signal_code_to_string(delivery_request.get_control_signal_code()),
                         precondition_to_string(qos.get_precondition()),
                         drop_strategy_to_string(qos.get_drop_strategy()));
            success = m_primary_output_port->push_request(delivery_request, qos);
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

        if (ret_read_frame == ReadFrameResult::END_OF_VIDEO) {
            // end of video, stop it
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "End of video, stopping");
            _async_stop();
        }
    }
}

} // namespace redoxi_works::video_readers::v2
