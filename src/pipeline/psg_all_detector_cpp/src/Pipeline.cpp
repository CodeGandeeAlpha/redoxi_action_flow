#include <psg_all_detector_cpp/Pipeline.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessReplyHandler.hpp>
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
using ModelResultPromise = std::promise<std::shared_ptr<PSGAllDetectorCppNode::OutputModelResult_t>>;
using ModelResultFuture = std::shared_future<std::shared_ptr<PSGAllDetectorCppNode::OutputModelResult_t>>;

struct PSGAllDetectorCppImpl {
    struct OutputModelResult {
        std::shared_ptr<ModelResultPromise> promise;
        ModelResultFuture future;
        std::shared_ptr<PSGAllDetectorCppNode::OutputSourceDataModel_t> source_data;
        ControlSignalCode control_signal_code;
    };

    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    //! buffer the model result
    tbb::concurrent_bounded_queue<OutputModelResult> m_model_result_buffer;

    //! task group for model result
    tbb::task_group m_model_result_task_group;

    // pull input, work on it and then send output
    using PullProcessSendHandler_t = redoxi_works::port_handlers::PullProcessSendHandler<PSGAllDetectorCppNode::InputPort_t::MasterSpec_t,
                                                                                         PSGAllDetectorCppNode::OutputPortModel_t::MasterSpec_t>;
    std::shared_ptr<PullProcessSendHandler_t> work_then_send_to_model_handler;
};

PSGAllDetectorCppNode::PSGAllDetectorCppNode(const std::string &name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(name, options)
{
}

PSGAllDetectorCppNode::~PSGAllDetectorCppNode()
{
    // wait for all requests to be processed
    if (m_primary_output_port_pipeline) {
        m_primary_output_port_pipeline->wait_for_all_requests();
    }
    if (m_primary_output_port_model) {
        m_primary_output_port_model->wait_for_all_requests();
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

int PSGAllDetectorCppNode::_start()
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);

    //! Start input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting psg detector in node");
    m_input_port->start();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port started");

    //! Start primary output port
    if (m_primary_output_port_pipeline) {
        auto ret = m_primary_output_port_pipeline->start();
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}] Failed to start primary output port pipeline, ret={}", __func__, ret);
            return ret;
        }
    }
    if (m_primary_output_port_model) {
        auto ret = m_primary_output_port_model->start();
        if (ret != 0) {
            RDX_RAISE_ERROR("[{}] Failed to start primary output port model, ret={}", __func__, ret);
            return ret;
        }
    }

    //! start ros time token
    {
        auto interval = runtime_config->document_interval;
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

int PSGAllDetectorCppNode::_stop()
{
    //! Stop input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Stopping psg detector node");
    m_input_port->stop();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port stopped");

    //! Stop primary output port
    if (m_primary_output_port_pipeline) {
        m_primary_output_port_pipeline->stop();
    }
    if (m_primary_output_port_model) {
        m_primary_output_port_model->stop();
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

void PSGAllDetectorCppNode::set_publish_to_debug_topic(bool enable)
{
    m_publish_to_debug_topic = enable;
    if (m_publish_to_debug_topic) {
        if (m_primary_output_port_pipeline) {
            m_primary_output_port_pipeline->set_publish_to_debug_topic(enable);
        }
        if (m_primary_output_port_model) {
            m_primary_output_port_model->set_publish_to_debug_topic(enable);
        }
    }
}

bool PSGAllDetectorCppNode::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int PSGAllDetectorCppNode::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
{
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(config);

    // parse the config into a string and print it
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "parse init config into a string");
    auto config_str = JS::serializeStruct(*init_config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "init config: {}", config_str);

    // create impl
    m_impl = _create_impl();

    //! config must have some downstream
    // RDX_ASSERT_CHECK_TRUE(!config->primary_output_spec.get_downstream_specs().empty(),
    //                       "[{}] init config must have at least one downstream", __func__);

    //! Initialize output ports
    auto primary_output_port_pipeline = _create_primary_output_port_pipeline(*init_config);
    if (!primary_output_port_pipeline) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port_pipeline = primary_output_port_pipeline;

    auto primary_output_port_model = _create_primary_output_port_model(*init_config);
    if (!primary_output_port_model) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port_model = primary_output_port_model;

    //! Create and initialize input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating input port");
    m_input_port = std::make_shared<InputPort_t>(this);
    m_input_port->init(init_config->input_port_config);

    //! Initialize debug publishers
    if (init_config->create_debug_pub) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "initialize debug publishers, pipeline enqueue topic={}, pipeline drop topic={}, model enqueue topic={}, model drop topic={}",
                     init_config->debug_pub_pipeline_enqueue_name,
                     init_config->debug_pub_pipeline_drop_name,
                     init_config->debug_pub_model_enqueue_name,
                     init_config->debug_pub_model_drop_name);
        auto debug_qos = DefaultParams::DebugPublisherQoS;
        m_pub_pipeline_enqueue.init(this, init_config->debug_pub_pipeline_enqueue_name, debug_qos);
        m_pub_pipeline_drop.init(this, init_config->debug_pub_pipeline_drop_name, debug_qos);
        m_pub_model_enqueue.init(this, init_config->debug_pub_model_enqueue_name, debug_qos);
        m_pub_model_drop.init(this, init_config->debug_pub_model_drop_name, debug_qos);
    }

    return 0;
}

int PSGAllDetectorCppNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(config);

    //! parse the config into a string and print it
    auto config_str = JS::serializeStruct(*runtime_config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "runtime config: {}", config_str);


    //! set callback on request enqueued to resize image if needed
    m_primary_output_port_pipeline->set_callback_on_request_enqueued([](DeliveryRequestPipeline_t &request) {
        // do nothing
        (void)request;
    });
    m_primary_output_port_model->set_callback_on_request_enqueued([](DeliveryRequestModel_t &request) {
        // do nothing
        (void)request;
    });

    //! set publish to debug topic
    set_publish_to_debug_topic(runtime_config->publish_to_debug_topic);

    RDX_INFO_DEV(this, __func__, false, "{}", "Creating frame request handler");
    _create_frame_request_handler(*runtime_config);

    return 0;
}

std::shared_ptr<PSGAllDetectorCppImpl> PSGAllDetectorCppNode::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGAllDetectorCppImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

PSGAllDetectorCppNode::DeliveryRequestPipeline_t
    PSGAllDetectorCppNode::_create_delivery_request(const OutputSourceDataPipeline_t &source_data,
                                                    std::optional<ControlSignalCode> control_signal_code)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    //! Create delivery request
    DeliveryRequestPipeline_t req;
    req.set_source_data(source_data);
    if (runtime_config->pipeline_request_policy.has_value()) {
        req.set_delivery_policy(*runtime_config->pipeline_request_policy);
    }
    if (control_signal_code.has_value()) {
        req.set_control_signal_code(*control_signal_code);
    }

    return req;
}

PSGAllDetectorCppNode::DeliveryRequestModel_t
    PSGAllDetectorCppNode::_create_delivery_request(const OutputSourceDataModel_t &source_data,
                                                    std::optional<ControlSignalCode> control_signal_code)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    //! Create delivery request
    DeliveryRequestModel_t req;
    req.set_source_data(source_data);
    if (runtime_config->model_request_policy.has_value()) {
        req.set_delivery_policy(*runtime_config->model_request_policy);
    }
    if (control_signal_code.has_value()) {
        req.set_control_signal_code(*control_signal_code);
    }
    return req;
}

std::shared_ptr<PSGAllDetectorCppNode::OutputPortPipeline_t>
    PSGAllDetectorCppNode::_create_primary_output_port_pipeline(const InitConfig_t &init_config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port pipeline");
    auto port = std::make_shared<OutputPortPipeline_t>(this);
    auto &port_config = init_config.output_port_pipeline_config;
    // RDX_ASSERT_CHECK_TRUE(!port_config.get_downstream_specs().empty(),
    //                       "[{}] port_config must have at least one downstream", __func__);
    port->init(port_config);

    // register callbacks
    // port->set_callback_on_deliver_task_begin([this](TargetDataPipeline_t &target_data, const DeliveryTaskPipeline_t &task) {
    //     return _on_delivery_task_begin(target_data, task.get_request());
    // });
    // port->set_callback_on_deliver_task_finish([this](TargetDataPipeline_t &target_data, const DeliveryTaskPipeline_t &task, const DeliveryResultPipeline_t &result) {
    //     return _on_delivery_task_finish(target_data, task.get_request(), result);
    // });
    // port->set_callback_on_deliver_to_downstream_finish([this](TargetDataPipeline_t &target_data,
    //                                                           SendResultPipeline_t &result,
    //                                                           const DeliveryRequestPipeline_t &request,
    //                                                           const DownstreamPipeline_t &ds) {
    //     return _on_deliver_to_downstream_finish(target_data, result, request, ds);
    // });

    return port;
}

std::shared_ptr<PSGAllDetectorCppNode::OutputPortModel_t>
    PSGAllDetectorCppNode::_create_primary_output_port_model(const InitConfig_t &init_config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port model");
    auto port = std::make_shared<OutputPortModel_t>(this);
    auto &port_config = init_config.output_port_model_config;
    // RDX_ASSERT_CHECK_TRUE(!port_config.get_downstream_specs().empty(),
    //                       "[{}] port_config must have at least one downstream", __func__);
    port->init(port_config);


    // register callbacks
    // port->set_callback_on_deliver_task_begin([this](TargetDataModel_t &target_data, const DeliveryTaskModel_t &task) {
    //     return _on_delivery_task_begin(target_data, task.get_request());
    // });
    // port->set_callback_on_deliver_task_finish([this](TargetDataModel_t &target_data, const DeliveryTaskModel_t &task, const DeliveryResultModel_t &result) {
    //     return _on_delivery_task_finish(target_data, task.get_request(), result);
    // });
    port->set_callback_on_deliver_to_downstream_finish([this](TargetDataModel_t &target_data,
                                                              SendResultModel_t &result,
                                                              const DeliveryRequestModel_t &request,
                                                              const DownstreamModel_t &ds) {
        return _on_deliver_to_downstream_finish(target_data, result, request, ds);
    });

    return port;
}

int PSGAllDetectorCppNode::_create_frame_request_handler(const RuntimeConfig_t &runtime_config)
{
    using ProcessHandler_t = PSGAllDetectorCppImpl::PullProcessSendHandler_t;
    using InputDataTrait_t = PSGAllDetectorCppNode::InputPort_t::ActionDataTrait_t;
    auto config = std::make_shared<ProcessHandler_t::InitConfig_t>();

    config->block_input_reading = runtime_config.enable_blocking_mode;
    config->block_resource_acquisition = runtime_config.enable_blocking_mode;

    auto enqueue_policy = runtime_config.model_enqueue_policy;
    m_impl->work_then_send_to_model_handler = std::make_shared<ProcessHandler_t>();
    auto process_handler = m_impl->work_then_send_to_model_handler;
    process_handler->init(m_input_port.get(), m_primary_output_port_model.get(),
                          nullptr, config, enqueue_policy);

    process_handler->on_process_input_data =
        [this](ProcessHandler_t::OutputRequest_t *output_request,
               std::optional<ProcessHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
               ProcessHandler_t::InputActionResult_t *action_result,
               std::shared_ptr<const InputSourceData_t> source_data,
               ProcessHandler_t::ResourceToken_t &resource) {
            // from input source data to output source data
            OutputSourceDataModel_t output_source_data;
            OutputSourceDataModel_t::FrameData_t frame_data;
            image_utils::FrameMediator fm(&source_data->get_goal()->document.frame_bundle.primary_frame);
            fm.to_cv_image_copy(frame_data.image);
            frame_data.metadata = source_data->get_goal()->document.frame_bundle.primary_frame.metadata;
            output_source_data.set_primary_frame(frame_data);

            auto goal_handle = source_data->get_goal_handle_future().get();
            auto control_signal_code = InputDataTrait_t::get_control_signal_code(*source_data->get_goal());
            RDX_INFO_DEV(this, __func__, true,
                         "on_process_input_data()中frame num: {}, control signal code: {}",
                         source_data->get_goal()->document.frame_bundle.primary_frame.metadata.frame_num, int(control_signal_code));


            // create delivery request
            auto delivery_request = _create_delivery_request(output_source_data, control_signal_code);
            *output_request = delivery_request;

            // fill the action result, nothing to do
            (void)action_result;

            (void)output_enqueue_policy;
            (void)resource;
            return 0;
        };
    return 0;
}

int PSGAllDetectorCppNode::_process_frame_request()
{
    auto ret = m_impl->work_then_send_to_model_handler->process_and_send();
    if (ret == PSGAllDetectorCppImpl::PullProcessSendHandler_t::ProcessResult::Error) {
        RDX_INFO_DEV(this, __func__, false, "Failed to process image request, error code: {}", int(ret));
        return -1;
    } else if (ret == PSGAllDetectorCppImpl::PullProcessSendHandler_t::ProcessResult::NoData) {
        //! No data available, skipping
        return 0;
    } else if (ret == PSGAllDetectorCppImpl::PullProcessSendHandler_t::ProcessResult::Success) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Successfully processed image request");
        return 0;
    } else if (ret == PSGAllDetectorCppImpl::PullProcessSendHandler_t::ProcessResult::NoResourceToken) {
        //! No resource token, skipping
        return 0;
    } else if (ret == PSGAllDetectorCppImpl::PullProcessSendHandler_t::ProcessResult::FailedToSend) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Failed to send image request to downstream, do you have a downstream?");
        return 0;
    } else {
        RDX_RAISE_ERROR("[f={}] Unexpected process result: {}", __func__, int(ret));
        return -1;
    }
}

void PSGAllDetectorCppNode::_step()
{
    if (m_input_port) {
        _process_frame_request();
    }

    // // 自己读取图片往后发送
    // OutputSourceDataModel_t output_model_source_data;
    // _read_frame(output_model_source_data, m_frame_number);
    // m_frame_number++;

    // // create delivery request
    // auto delivery_request = _create_delivery_request(output_model_source_data);

    // // this is used for logging
    // auto msg_uuid = output_model_source_data.get_uuid();

    // // get qos, controls how to retry and drop frames
    // auto &qos = m_runtime_config->model_enqueue_policy;
    // auto success = m_primary_output_port_model->push_request(delivery_request, qos);

    // if (success) {
    //     RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
    //                  "[msg_uuid={}] success to push request",
    //                  boost::uuids::to_string(msg_uuid));
    // } else {
    //     RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
    //                  "[msg_uuid={}] failed to push request",
    //                  boost::uuids::to_string(msg_uuid));
    // }

    // // FIXME: debug only
    // // wait for all requests to be processed, not necessary
    // m_primary_output_port_model->wait_for_all_requests();
}

int PSGAllDetectorCppNode::_on_deliver_to_downstream_finish(TargetDataModel_t &target_data,
                                                            SendResultModel_t &result,
                                                            const DeliveryRequestModel_t &request,
                                                            const DownstreamModel_t &ds)
{
    (void)target_data;

    // 1. 创建modelresult
    PSGAllDetectorCppImpl::OutputModelResult output_model_result;

    // 2. 绑定promise和future
    output_model_result.promise = std::make_shared<ModelResultPromise>();
    output_model_result.future = output_model_result.promise->get_future().share();
    output_model_result.source_data = std::make_shared<OutputSourceDataModel_t>(request.get_source_data());
    output_model_result.control_signal_code = request.get_control_signal_code();

    // 3. 将output_model_result推送到buffer中
    m_impl->m_model_result_buffer.push(output_model_result);

    // 4. 创建task 在tbb run中将结果写入promise
    auto promise = output_model_result.promise;
    //! 通过值捕获需要的数据
    auto ds_copy = ds;
    auto result_copy = result;
    m_impl->m_model_result_task_group.run([ds = ds_copy,
                                           result = result_copy,
                                           promise]() {
        auto goal_handle = result.goal_handle_future.get();
        if (goal_handle) {
            auto action_result = ds.get_action_client()->async_get_result(goal_handle).get().result;
            // 将action result写入promise
            auto output_model_result = std::make_shared<PSGAllDetectorCppNode::OutputModelResult_t>();
            output_model_result->detections = action_result->detections;
            output_model_result->x_return = action_result->x_return;
            promise->set_value(output_model_result);
        } else {
            promise->set_value(nullptr);
        }
    });

    return 0;
}

//! 将document中的raw image转换为带有检测框的debug image
sensor_msgs::msg::Image PSGAllDetectorCppNode::_create_debug_image(const psg_private_msgs::msg::PsgDocument &document)
{
    //! 转换raw image到cv::Mat
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始转换primary_frame到cv::Mat", 0);
    image_utils::FrameMediator fm(&document.frame_bundle.primary_frame);
    cv::Mat cv_image;
    fm.to_cv_image_copy(cv_image);
    auto encoding = fm.get_encoding();

    //! 为不同类别设置不同颜色
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "设置类别颜色映射", 0);
    std::map<int, cv::Scalar> class_colors = {
        {0, cv::Scalar(0, 255, 0)}, // 人-绿色
        {1, cv::Scalar(0, 0, 255)}, // 头-红色
        {2, cv::Scalar(255, 0, 0)}, // 脸-蓝色
    };

    //! 在图像上画bbox
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始在图像上绘制检测框, 共{}个检测结果", document.detections.size());
    for (const auto &detection : document.detections) {
        //! 获取bbox坐标
        int x = static_cast<int>(detection.bbox.x);
        int y = static_cast<int>(detection.bbox.y);
        int width = static_cast<int>(detection.bbox.width);
        int height = static_cast<int>(detection.bbox.height);

        //! 获取类别对应的颜色
        cv::Scalar color = class_colors[detection.category];

        //! 画框
        cv::rectangle(cv_image,
                      cv::Point(x, y),
                      cv::Point(x + width, y + height),
                      color, 2);

        //! 添加类别标签
        std::string label = std::to_string(detection.category) + " " +
                            std::to_string(detection.confidence).substr(0, 4);
        cv::putText(cv_image, label,
                    cv::Point(x, y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    color, 2);

        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                      "绘制检测框 - 类别:{}, 置信度:{:.2f}, 位置:[{}, {}, {}, {}]",
                      detection.category, detection.confidence, x, y, width, height);
    }

    //! 转回sensor_msgs/Image
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始转换回sensor_msgs/Image", 0);
    sensor_msgs::msg::Image debug_image;
    image_utils::FrameMediator fm_cv_image(cv_image, encoding);
    fm_cv_image.to_image_msg(debug_image);
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "完成debug图像创建, 大小: {}x{}", debug_image.width, debug_image.height);
    return debug_image;
}

void PSGAllDetectorCppNode::_get_model_result()
{
    // 1. 从buffer中取出model result, 如果buffer为空，则等待
    PSGAllDetectorCppImpl::OutputModelResult output_model_result;
    m_impl->m_model_result_buffer.pop(output_model_result);
    // 2. 等待结果
    auto result = output_model_result.future.get();
    // 3. 如果结果不为空，则构造output source data，并推送到output port pipeline
    if (result) {
        auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
        auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
        // create output source data
        OutputSourceDataPipeline_t output_pipeline_source_data;
        psg_private_msgs::msg::PsgDocument document;
        output_model_result.source_data->get_primary_frame().to_frame_msg(document.frame_bundle.primary_frame);
        document.detections = result->detections;

        // 根据detections中的keypoints的头点，构建头的bbox
        for (const auto &detection : result->detections) {
            auto head_keypoint = detection.keypoints.keypoints_2[0];
            auto body_width = detection.bbox.width;
            auto body_height = detection.bbox.height;
            auto head_bbox = cv::Rect(head_keypoint.x - body_width / 3 / 2,
                                      head_keypoint.y - body_height / 6 / 2,
                                      body_width / 3,
                                      body_height / 6);
            redoxi_public_msgs::msg::Detection head_detection;
            head_detection.bbox.x = head_bbox.x;
            head_detection.bbox.y = head_bbox.y;
            head_detection.bbox.width = head_bbox.width;
            head_detection.bbox.height = head_bbox.height;
            head_detection.category = 1;
            head_detection.confidence = 1.0;
            document.detections.push_back(head_detection);
        }

        output_pipeline_source_data.set_document(document);

        // test: 输出所有document.detections的bbox和类别
        for (const auto &det : document.detections) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "bbox: [{}, {}, {}, {}], category: {}",
                         det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height, det.category);
        }

        // create pipeline delivery request
        auto control_signal_code = output_model_result.control_signal_code;
        auto delivery_request = _create_delivery_request(output_pipeline_source_data, control_signal_code);

        // test log
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "control signal code: {}", int(control_signal_code));
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "document frame num: {}, control signal code: {}",
                     document.frame_bundle.primary_frame.metadata.frame_num, int(document.x_control.code));


        // push to output port pipeline
        // this is used for logging
        auto msg_uuid = output_pipeline_source_data.get_uuid();

        // get qos, controls how to retry and drop frames
        auto &qos = runtime_config->pipeline_enqueue_policy;
        auto success = m_primary_output_port_pipeline->push_request(delivery_request, qos);

        if (success) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "[msg_uuid={}] success to push request",
                         boost::uuids::to_string(msg_uuid));

            std::string det_str;
            for (const auto &det : result->detections) {
                det_str += fmt::format("body bbox: [{}, {}, {}, {}] ",
                                       det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height);
            }
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "[msg_uuid={}] got detection result: {}",
                         boost::uuids::to_string(output_model_result.source_data->get_uuid()), det_str);

            if (init_config->create_debug_pub) {
                auto debug_image = _create_debug_image(document);
                m_pub_model_enqueue.publish(debug_image, "");
            }
        } else {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "[msg_uuid={}] failed to push request",
                         boost::uuids::to_string(msg_uuid));
        }

        // // FIXME: debug only
        // // wait for all requests to be processed, not necessary
        // m_primary_output_port_pipeline->wait_for_all_requests();
    }
}

// int PSGAllDetectorCppNode::_read_frame(OutputSourceDataModel_t &data, std::atomic<int64_t> &frame_number)
// {
//     auto frame_size = cv::Size(1920, 1080);
//     if (frame_size.empty()) {
//         RDX_RAISE_ERROR("[{}][_read_frame()] output_image_size is not set", this->get_name());
//     }

//     //! Generate a random frame with the UUID text
//     cv::Mat random_frame = cv::imread("data/ori_img.jpg");
//     auto uuid = data.get_uuid();
//     auto frame_text = fmt::format("{}\nFrame Number: {}", boost::uuids::to_string(uuid), frame_number.load());
//     // random_image_with_text(random_frame, frame_size, frame_text);


//     psg_private_msgs::msg::PsgDocument doc_msg;
//     // convert image to ROS message
//     cv_bridge::CvImage cv_bridge_image;
//     cv_bridge_image.image = random_frame;
//     cv_bridge_image.encoding = sensor_msgs::image_encodings::BGR8;
//     cv_bridge_image.toImageMsg(doc_msg.frame.raw_image);
//     doc_msg.frame.metadata.frame_num = frame_number;
//     data.set_document(doc_msg);

//     frame_number++;
//     return 0;
// }

} // namespace redoxi_works
