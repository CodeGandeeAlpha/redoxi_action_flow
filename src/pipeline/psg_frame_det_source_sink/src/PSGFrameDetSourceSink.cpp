#include <psg_frame_det_source_sink/PSGFrameDetSourceSink.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_samples_lib/random_image.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <json_struct/json_struct.h>
#include <tbb/concurrent_queue.h>

// #include <tbb/task_group.h>

#define PRINT_THREAD_ID_IN_LOG (true)

namespace redoxi_works
{
// shared_ptr is used to indicate whether the result is available
// 如果我在request创建的时候就把promise和future绑定在一起，那么当request发送失败的时候我也可以把结果设置为nullptr来表示失败
using ModelResultPromise = std::promise<std::shared_ptr<PSGFrameDetSourceSink::OutputResult_t>>;
using ModelResultFuture = std::shared_future<std::shared_ptr<PSGFrameDetSourceSink::OutputResult_t>>;

struct PSGFrameDetSourceSinkImpl {
    struct OutputModelResult {
        std::shared_ptr<ModelResultPromise> promise;
        ModelResultFuture future;
        std::shared_ptr<PSGFrameDetSourceSink::SourceData_t> source_data;
    };

    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    //! buffer the model result
    tbb::concurrent_bounded_queue<OutputModelResult> m_model_result_buffer;

    //! task group for model result
    tbb::task_group m_model_result_task_group;
};

PSGFrameDetSourceSink::~PSGFrameDetSourceSink()
{
    // wait for all requests to be processed
    if (m_primary_output_port) {
        m_primary_output_port->wait_for_all_requests();
    }

    // stop ros time token
    if (m_impl->m_ros_time_token) {
        m_impl->m_ros_time_token->stop();
    }

    // stop get model result thread
    m_get_model_result_thread_running = false;
    if (m_get_model_result_thread != nullptr && m_get_model_result_thread->joinable()) {
        m_get_model_result_thread->join();
    }

    // destroy impl task group
    m_impl->m_model_result_task_group.wait();
}

int PSGFrameDetSourceSink::_start()
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    //! Start primary output port
    if (m_primary_output_port) {
        auto ret = m_primary_output_port->start();
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}] Failed to start primary output port pipeline, ret={}", __func__, ret);
            return ret;
        }
    }

    //! start ros time token
    {
        auto interval = runtime_config->step_interval;
        m_impl->m_ros_time_token->start(interval);
    }

    //! start get model result thread
    m_get_model_result_thread_running = true;
    m_get_model_result_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok() && m_get_model_result_thread_running) {
            _get_model_result();
        }
    });

    return 0;
}

int PSGFrameDetSourceSink::_stop()
{
    //! Stop primary output port
    if (m_primary_output_port) {
        m_primary_output_port->stop();
    }
    //! stop ros time token
    m_impl->m_ros_time_token->stop();

    //! Stop get model result thread
    m_get_model_result_thread_running = false;
    if (m_get_model_result_thread != nullptr && m_get_model_result_thread->joinable()) {
        m_get_model_result_thread->join();
    }

    //! destroy impl task group
    m_impl->m_model_result_task_group.wait();

    return 0;
}

void PSGFrameDetSourceSink::set_publish_to_debug_topic(bool enable)
{
    m_publish_to_debug_topic = enable;
    if (m_publish_to_debug_topic) {
        if (m_primary_output_port) {
            m_primary_output_port->set_publish_to_debug_topic(enable);
        }
    }
}

bool PSGFrameDetSourceSink::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int PSGFrameDetSourceSink::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
{
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(config);

    // parse the config into a string and print it
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "parse init config into a string");
    auto config_str = JS::serializeStruct(*config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "init config: {}", config_str);

    // create impl
    m_impl = _create_impl();

    //! config must have some downstream
    // RDX_ASSERT_CHECK_TRUE(!config->primary_output_spec.get_downstream_specs().empty(),
    //                       "[{}] init config must have at least one downstream", __func__);

    //! Initialize output ports
    auto primary_output_port = _create_primary_output_port(*init_config);
    if (!primary_output_port) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port = primary_output_port;

    //! Initialize debug publishers
    if (init_config->create_debug_pub) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "initialize debug publishers, task enqueue topic={}, task drop topic={}",
                     init_config->debug_pub_task_enqueue_name,
                     init_config->debug_pub_task_drop_name);
        auto debug_qos = DefaultParams::get_debug_publisher_qos();
        m_pub_task_enqueue.init(this, init_config->debug_pub_task_enqueue_name, debug_qos);
        m_pub_task_drop.init(this, init_config->debug_pub_task_drop_name, debug_qos);
    }

    return 0;
}

int PSGFrameDetSourceSink::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(config);

    //! parse the config into a string and print it
    auto config_str = JS::serializeStruct(*config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "runtime config: {}", config_str);

    //! set callback on request enqueued to resize image if needed
    m_primary_output_port->set_callback_on_request_enqueued([](DeliveryRequest_t &request) {
        // do nothing
    });

    //! set publish to debug topic
    set_publish_to_debug_topic(runtime_config->publish_to_debug_topic);

    return 0;
}

std::shared_ptr<PSGFrameDetSourceSinkImpl> PSGFrameDetSourceSink::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGFrameDetSourceSinkImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

PSGFrameDetSourceSink::DeliveryRequest_t
    PSGFrameDetSourceSink::_create_delivery_request(const SourceData_t &source_data)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    //! Create delivery request
    DeliveryRequest_t req;
    req.set_source_data(source_data);
    if (runtime_config->frame_request_policy.has_value()) {
        req.set_delivery_policy(*runtime_config->frame_request_policy);
    }

    return req;
}


std::shared_ptr<PSGFrameDetSourceSink::OutputPort_t>
    PSGFrameDetSourceSink::_create_primary_output_port(const InitConfig_t &init_config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port pipeline");
    auto port = std::make_shared<OutputPort_t>(this);
    auto &port_config = init_config.primary_output_spec;
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
    port->set_callback_on_deliver_to_downstream_finish([this](TargetData_t &target_data,
                                                              SendResult_t &result,
                                                              const DeliveryRequest_t &request,
                                                              const Downstream_t &ds) {
        return _on_deliver_to_downstream_finish(target_data, result, request, ds);
    });

    return port;
}

void PSGFrameDetSourceSink::_step()
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    // 从input port pipeline获取数据，创建delivery request，并推送到output port model,
    // 从input port model获取数据，放到detections buffer中去
    if (get_status() != NodeStatusCode::STARTED) {
        return;
    }

    SourceData_t source_data;
    auto ret = _read_frame(source_data, m_frame_number);
    if (ret != 0) {
        RDX_LOG_ERROR(this, __func__, true, "{}", "Failed to read frame");
        return;
    }


    // create delivery request
    auto delivery_request = _create_delivery_request(source_data);

    // this is used for logging
    auto msg_uuid = source_data.get_uuid();

    // get qos, controls how to retry and drop frames
    auto &qos = runtime_config->frame_enqueue_policy;
    bool success = m_primary_output_port->push_request(delivery_request, qos);

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

int PSGFrameDetSourceSink::_on_deliver_to_downstream_finish(TargetData_t &target_data,
                                                            SendResult_t &result,
                                                            const DeliveryRequest_t &request,
                                                            const Downstream_t &ds)
{
    //! 1. 创建modelresult
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "Creating model result");
    PSGFrameDetSourceSinkImpl::OutputModelResult output_model_result;

    //! 2. 绑定promise和future
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "Binding promise and future");
    output_model_result.promise = std::make_shared<ModelResultPromise>();
    output_model_result.future = output_model_result.promise->get_future().share();
    output_model_result.source_data = std::make_shared<SourceData_t>(request.get_source_data());

    //! 3. 将output_model_result推送到buffer中
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "[msg_uuid={}] Pushing model result to buffer",
                 boost::uuids::to_string(request.get_source_data().get_uuid()));
    m_impl->m_model_result_buffer.push(output_model_result);

    //! 4. 创建task 在tbb run中将结果写入promise
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "[msg_uuid={}] Creating task to write result to promise",
                 boost::uuids::to_string(request.get_source_data().get_uuid()));
    auto promise = output_model_result.promise;
    //! 通过值捕获需要的数据
    auto ds_copy = ds;
    auto result_copy = result;
    m_impl->m_model_result_task_group.run([ds = std::move(ds_copy),
                                           result = std::move(result_copy),
                                           promise,
                                           this]() {
        auto goal_handle = result.goal_handle_future.get();
        if (goal_handle) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "Got valid goal handle, getting action result");
            auto action_result = ds.get_action_client()->async_get_result(goal_handle).get().result; // TODO: 卡在这里了，由于后面的python节点没有返回正常的结果
            //! 将action result写入promise
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "Writing action result to promise");
            auto output_model_result = std::make_shared<PSGFrameDetSourceSink::OutputResult_t>();
            output_model_result->detections = action_result->detections;
            output_model_result->x_return = action_result->x_return;
            promise->set_value(output_model_result);
        } else {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "Got invalid goal handle, setting promise to null");
            promise->set_value(nullptr);
        }
    });

    return 0;
}
void PSGFrameDetSourceSink::_get_model_result()
{
    //! 1. 从buffer中取出model result, 如果buffer为空，则等待
    PSGFrameDetSourceSinkImpl::OutputModelResult output_model_result;
    m_impl->m_model_result_buffer.pop(output_model_result);

    //! 2. 等待结果
    auto result = output_model_result.future.get();

    //! 3. 如果结果不为空，则构造output source data，并推送到output port pipeline
    if (result) {
        auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
        auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
        //! 构造string消息
        std_msgs::msg::String msg;
        std::string det_str;
        for (const auto &det : result->detections) {
            det_str += fmt::format("body bbox: [{}, {}, {}, {}] ",
                                   det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height);
        }
        msg.data = det_str;

        //! 通过debug publisher发布
        if (init_config->create_debug_pub) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "[msg_uuid={}] Publishing detection result: {}",
                         boost::uuids::to_string(output_model_result.source_data->get_uuid()), det_str);
            m_pub_task_enqueue.publish(msg);
        }
    } else {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "[msg_uuid={}] Model result is null",
                     boost::uuids::to_string(output_model_result.source_data->get_uuid()));
    }
}

int PSGFrameDetSourceSink::_read_frame(SourceData_t &data, std::atomic<int64_t> &frame_number)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    auto frame_size = runtime_config->output_image_size;
    if (frame_size.empty()) {
        RDX_RAISE_ERROR("[{}][_read_frame()] output_image_size is not set", this->get_name());
    }

    //! Generate a random frame with the UUID text
    cv::Mat random_frame = cv::imread("data/ori_img.jpg");
    auto uuid = data.get_uuid();
    auto frame_text = fmt::format("{}\nFrame Number: {}", boost::uuids::to_string(uuid), frame_number.load());
    // random_image_with_text(random_frame, frame_size, frame_text);


    psg_private_msgs::msg::PsgDocument doc_msg;
    image_utils::FrameMediator fm(random_frame);
    fm.to_image_msg(doc_msg.frame_bundle.primary_frame.raw_image);
    doc_msg.frame_bundle.primary_frame.metadata.frame_num = frame_number;
    data.set_document(doc_msg);

    frame_number++;
    return 0;
}

} // namespace redoxi_works
