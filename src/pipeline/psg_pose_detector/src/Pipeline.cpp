#include <psg_pose_detector/Pipeline.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
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
using ModelResultPromise = std::promise<std::shared_ptr<PSGPoseDetectorNode::OutputModelResult_t>>;
using ModelResultFuture = std::shared_future<std::shared_ptr<PSGPoseDetectorNode::OutputModelResult_t>>;

struct PSGPoseDetectorImpl {
    struct OutputModelResult {
        std::shared_ptr<ModelResultPromise> promise;
        ModelResultFuture future;
        std::shared_ptr<PSGPoseDetectorNode::OutputSourceDataModel_t> source_data;
        ControlSignalCode control_signal_code;
    };

    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    //! buffer the model result
    tbb::concurrent_bounded_queue<OutputModelResult> m_model_result_buffer;

    //! task group for model result
    tbb::task_group m_model_result_task_group;

    //! 记录document的异步字典
    using DocumentMap_t = boost::synchronized_value<std::map<int, std::shared_ptr<psg_private_msgs::msg::PsgDocument>>>;
    DocumentMap_t m_document_map;

    // pull input, work on it and then send output
    using PullProcessSendHandler_t = redoxi_works::port_handlers::PullProcessSendHandler<PSGPoseDetectorNode::InputPort_t::MasterSpec_t,
                                                                                         PSGPoseDetectorNode::OutputPortModel_t::MasterSpec_t>;
    std::shared_ptr<PullProcessSendHandler_t> work_then_send_to_model_handler;
};

PSGPoseDetectorNode::PSGPoseDetectorNode(const std::string &name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(name, options)
{
}

PSGPoseDetectorNode::~PSGPoseDetectorNode()
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

int PSGPoseDetectorNode::_start()
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

    RDX_INFO_DEV(this, __func__, false, "{}", "psg pose detector node started");

    return 0;
}

int PSGPoseDetectorNode::_stop()
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

void PSGPoseDetectorNode::set_publish_to_debug_topic(bool enable)
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

bool PSGPoseDetectorNode::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int PSGPoseDetectorNode::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
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

int PSGPoseDetectorNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
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


    RDX_INFO_DEV(this, __func__, false, "{}", "Creating detections request handler");
    _create_detections_request_handler(*runtime_config);

    return 0;
}

std::shared_ptr<PSGPoseDetectorImpl> PSGPoseDetectorNode::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGPoseDetectorImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

PSGPoseDetectorNode::DeliveryRequestPipeline_t
    PSGPoseDetectorNode::_create_delivery_request(const OutputSourceDataPipeline_t &source_data,
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

PSGPoseDetectorNode::DeliveryRequestModel_t
    PSGPoseDetectorNode::_create_delivery_request(const OutputSourceDataModel_t &source_data,
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

std::shared_ptr<PSGPoseDetectorNode::OutputPortPipeline_t>
    PSGPoseDetectorNode::_create_primary_output_port_pipeline(const InitConfig_t &init_config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port pipeline");
    auto port = std::make_shared<OutputPortPipeline_t>(this);
    auto &port_config = init_config.output_port_pipeline_config;
    // RDX_ASSERT_CHECK_TRUE(!port_config.get_downstream_specs().empty(),
    //                       "[{}] port_config must have at least one downstream", __func__);
    port->init(port_config);

    // register callbacks
    port->set_callback_on_deliver_task_begin([this](TargetDataPipeline_t &target_data, const DeliveryTaskPipeline_t &task) {
        return _on_delivery_task_begin(target_data, task.get_request());
    });
    port->set_callback_on_deliver_task_finish([this](TargetDataPipeline_t &target_data, const DeliveryTaskPipeline_t &task, const DeliveryResultPipeline_t &result) {
        return _on_delivery_task_finish(target_data, task.get_request(), result);
    });
    port->set_callback_on_deliver_to_downstream_finish([this](TargetDataPipeline_t &target_data,
                                                              SendResultPipeline_t &result,
                                                              const DeliveryRequestPipeline_t &request,
                                                              const DownstreamPipeline_t &ds) {
        return _on_deliver_to_downstream_finish(target_data, result, request, ds);
    });

    return port;
}

std::shared_ptr<PSGPoseDetectorNode::OutputPortModel_t>
    PSGPoseDetectorNode::_create_primary_output_port_model(const InitConfig_t &init_config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port model");
    auto port = std::make_shared<OutputPortModel_t>(this);
    auto &port_config = init_config.output_port_model_config;
    // RDX_ASSERT_CHECK_TRUE(!port_config.get_downstream_specs().empty(),
    //                       "[{}] port_config must have at least one downstream", __func__);
    port->init(port_config);


    // register callbacks
    port->set_callback_on_deliver_task_begin([this](TargetDataModel_t &target_data, const DeliveryTaskModel_t &task) {
        return _on_delivery_task_begin(target_data, task.get_request());
    });
    port->set_callback_on_deliver_task_finish([this](TargetDataModel_t &target_data, const DeliveryTaskModel_t &task, const DeliveryResultModel_t &result) {
        return _on_delivery_task_finish(target_data, task.get_request(), result);
    });
    port->set_callback_on_deliver_to_downstream_finish([this](TargetDataModel_t &target_data,
                                                              SendResultModel_t &result,
                                                              const DeliveryRequestModel_t &request,
                                                              const DownstreamModel_t &ds) {
        return _on_deliver_to_downstream_finish(target_data, result, request, ds);
    });

    return port;
}

int PSGPoseDetectorNode::_create_detections_request_handler(const RuntimeConfig_t &runtime_config)
{
    using ProcessHandler_t = PSGPoseDetectorImpl::PullProcessSendHandler_t;
    using InputDataTrait_t = PSGPoseDetectorNode::InputPort_t::ActionDataTrait_t;
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
            // 将document数据放入document map中
            m_impl->m_document_map.synchronize()->insert({source_data->get_goal()->document.frame_bundle.primary_frame.metadata.frame_num,
                                                          std::make_shared<psg_private_msgs::msg::PsgDocument>(source_data->get_goal()->document)});

            //! 从输入数据创建输出数据
            RDX_INFO_DEV(this, __func__, true, "{}", "开始从输入数据创建输出数据");
            OutputSourceDataModel_t output_model_source_data;
            output_model_source_data.set_frame_bundle(source_data->get_goal()->document.frame_bundle);
            output_model_source_data.set_detections(source_data->get_goal()->document.detections);

            //! 根据种类挑选出body的detections，并记录其在document中的索引
            RDX_INFO_DEV(this, __func__, true, "{}", "开始筛选body类型的检测框");
            std::vector<size_t> body_detections_indices;
            for (size_t i = 0; i < source_data->get_goal()->document.detections.size(); ++i) {
                if (source_data->get_goal()->document.detections[i].category == 0) { // 0: body, 1: head, 2: face
                    body_detections_indices.push_back(i);
                }
            }
            RDX_INFO_DEV(this, __func__, true, "找到{}个body检测框", body_detections_indices.size());
            output_model_source_data.set_detections_indices(body_detections_indices);

            //! 获取控制信号
            RDX_INFO_DEV(this, __func__, true, "{}", "获取控制信号");
            auto goal_handle = source_data->get_goal_handle_future().get();
            auto control_signal_code = InputDataTrait_t::get_control_signal_code(*source_data->get_goal());
            RDX_INFO_DEV(this, __func__, true,
                         "on_process_input_data()中frame num: {}, control signal code: {}",
                         source_data->get_goal()->document.frame_bundle.primary_frame.metadata.frame_num, int(control_signal_code));
            //! 创建传输请求
            RDX_INFO_DEV(this, __func__, true, "{}", "创建传输请求");
            auto delivery_request = _create_delivery_request(output_model_source_data, control_signal_code);
            *output_request = delivery_request;

            //! 填充动作结果(无需操作)
            (void)action_result;

            (void)output_enqueue_policy;
            (void)resource;
            RDX_INFO_DEV(this, __func__, true, "{}", "处理完成");
            return 0;
        };
    return 0;
}

int PSGPoseDetectorNode::_process_detections_request()
{
    auto ret = m_impl->work_then_send_to_model_handler->process_and_send();
    if (ret == PSGPoseDetectorImpl::PullProcessSendHandler_t::ProcessResult::Error) {
        RDX_INFO_DEV(this, __func__, false, "Failed to process image request, error code: {}", int(ret));
        return -1;
    } else if (ret == PSGPoseDetectorImpl::PullProcessSendHandler_t::ProcessResult::NoData) {
        //! No data available, skipping
        // RDX_INFO_DEV(this, __func__, false, "{}", "No data available, skipping");
        return 0;
    } else if (ret == PSGPoseDetectorImpl::PullProcessSendHandler_t::ProcessResult::Success) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Successfully processed image request");
        return 0;
    } else if (ret == PSGPoseDetectorImpl::PullProcessSendHandler_t::ProcessResult::NoResourceToken) {
        RDX_INFO_DEV(this, __func__, false, "{}", "No resource token, skipping");
        return 0;
    } else if (ret == PSGPoseDetectorImpl::PullProcessSendHandler_t::ProcessResult::FailedToSend) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Failed to send image request to downstream, do you have a downstream?");
        return 0;
    } else {
        RDX_RAISE_ERROR("[f={}] Unexpected process result: {}", __func__, int(ret));
        return -1;
    }
}

void PSGPoseDetectorNode::_step()
{
    if (m_input_port) {
        _process_detections_request();
    }
}

int PSGPoseDetectorNode::_on_deliver_to_downstream_finish(TargetDataModel_t &target_data,
                                                          SendResultModel_t &result,
                                                          const DeliveryRequestModel_t &request,
                                                          const DownstreamModel_t &ds)
{
    (void)target_data;

    //! 1. 创建modelresult
    PSGPoseDetectorImpl::OutputModelResult output_model_result;

    //! 2. 绑定promise和future
    output_model_result.promise = std::make_shared<ModelResultPromise>();
    output_model_result.future = output_model_result.promise->get_future().share();
    output_model_result.source_data = std::make_shared<OutputSourceDataModel_t>(request.get_source_data());
    output_model_result.control_signal_code = request.get_control_signal_code();

    //! 3. 将output_model_result推送到buffer中
    m_impl->m_model_result_buffer.push(output_model_result);

    //! 4. 创建task 在tbb run中将结果写入promise
    auto promise = output_model_result.promise;
    //! 通过值捕获需要的数据
    auto ds_copy = ds;
    auto result_copy = result;
    m_impl->m_model_result_task_group.run([ds = ds_copy,
                                           result = result_copy,
                                           promise,
                                           this]() {
        auto goal_handle = result.goal_handle_future.get();
        if (goal_handle) {
            auto action_result = ds.get_action_client()->async_get_result(goal_handle).get().result;
            // 将action result写入promise
            auto output_model_result = std::make_shared<PSGPoseDetectorNode::OutputModelResult_t>();
            output_model_result->keypoints = action_result->keypoints;
            output_model_result->x_return = action_result->x_return;
            output_model_result->is_matched_by_uid = action_result->is_matched_by_uid;
            promise->set_value(output_model_result);
        } else {
            promise->set_value(nullptr);
        }
    });

    return 0;
}

//! 将document中的raw image转换为带有关键点的debug image
sensor_msgs::msg::Image PSGPoseDetectorNode::_create_debug_image(const psg_private_msgs::msg::PsgDocument &document)
{
    //! 转换raw image到cv::Mat
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始转换raw image到cv::Mat", 0);
    cv::Mat cv_image;
    image_utils::FrameMediator fm(&document.frame_bundle.primary_frame);
    fm.to_cv_image_copy(cv_image);

    //! 为关键点设置颜色
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "设置关键点颜色", 0);
    cv::Scalar keypoint_color(0, 255, 0); // 绿色
    cv::Scalar line_color(255, 255, 0);   // 黄色

    //! 在图像上画关键点和骨架连接
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始在图像上绘制关键点, 共{}个检测结果", document.detections.size());
    for (const auto &detection : document.detections) {
        const auto &keypoints = detection.keypoints;

        //! 在访问数组或指针前添加检查
        if (keypoints.keypoints_2.empty() || keypoints.confidence.empty()) {
            continue;
        }

        //! 画出17个关键点
        for (size_t i = 0; i < keypoints.keypoints_2.size(); i++) {
            if (keypoints.confidence[i] > 0.3) { // 只画置信度大于0.3的点
                //! 记录关键点的位置和置信度
                RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                              "绘制关键点[{}] - 位置:[{}, {}], 置信度:{}",
                              i, keypoints.keypoints_2[i].x, keypoints.keypoints_2[i].y, keypoints.confidence[i]);
                cv::circle(cv_image,
                           cv::Point(keypoints.keypoints_2[i].x, keypoints.keypoints_2[i].y),
                           3, keypoint_color, -1);
            }
        }

        //! 画出骨架连接
        //! COCO数据集的17个关键点连接对
        const std::vector<std::pair<int, int>> skeleton = {
            {5, 7}, {7, 9}, {6, 8}, {8, 10}, // 手臂
            {11, 13},
            {13, 15},
            {12, 14},
            {14, 16}, // 腿
            {5, 6},
            {5, 11},
            {6, 12},  // 躯干
            {11, 12}, // 臀部
            {1, 2},
            {1, 3},
            {2, 4},
            {3, 5},
            {4, 6} // 头部和肩膀
        };

        for (const auto &bone : skeleton) {
            if (keypoints.confidence[bone.first] > 0.3 && keypoints.confidence[bone.second] > 0.3) {
                //! 记录骨架连接的起点和终点
                RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                              "绘制骨架连接 - 从关键点[{},{}]到关键点[{},{}]",
                              keypoints.keypoints_2[bone.first].x, keypoints.keypoints_2[bone.first].y,
                              keypoints.keypoints_2[bone.second].x, keypoints.keypoints_2[bone.second].y);
                cv::line(cv_image,
                         cv::Point(keypoints.keypoints_2[bone.first].x, keypoints.keypoints_2[bone.first].y),
                         cv::Point(keypoints.keypoints_2[bone.second].x, keypoints.keypoints_2[bone.second].y),
                         line_color, 2);
            }
        }

        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                      "完成关键点绘制 - 检测框位置:[{}, {}, {}, {}]",
                      detection.bbox.x, detection.bbox.y, detection.bbox.width, detection.bbox.height);
    }

    //! 转回sensor_msgs/Image
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始转换回sensor_msgs/Image", 0);
    sensor_msgs::msg::Image debug_image;
    image_utils::FrameMediator fm_cv_image(cv_image, "bgr8");
    fm_cv_image.to_image_msg(debug_image);
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "完成debug图像创建, 大小: {}x{}", debug_image.width, debug_image.height);
    return debug_image;
}

void PSGPoseDetectorNode::_get_model_result()
{
    //! 1. 从buffer中取出model result, 如果buffer为空，则等待
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "开始从buffer中取出model result");
    PSGPoseDetectorImpl::OutputModelResult output_model_result;
    m_impl->m_model_result_buffer.pop(output_model_result);
    RDX_LOG_DEBUG(this, __func__, "{}", "成功从buffer中取出model result");

    //! 2. 等待结果
    RDX_LOG_DEBUG(this, __func__, "{}", "开始等待model result future");
    auto result = output_model_result.future.get();
    RDX_LOG_DEBUG(this, __func__, "got model result, size: {}", result->keypoints.size());

    //! 3. 如果结果不为空，则构造output source data，并推送到output port pipeline
    if (result) {
        auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
        auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);

        RDX_LOG_DEBUG(this, __func__, "{}", "开始构造output source data");
        // create output source data
        OutputSourceDataPipeline_t output_pipeline_source_data;
        // 根据frame_number获取document
        RDX_LOG_DEBUG(this, __func__, "{}", "开始从document map中获取document");
        auto document = m_impl->m_document_map.synchronize()->at(output_model_result.source_data->get_frame_bundle().primary_frame.metadata.frame_num);
        // 删掉字典中的document
        RDX_LOG_DEBUG(this, __func__, "{}", "开始从document map中删除document");
        m_impl->m_document_map.synchronize()->erase(output_model_result.source_data->get_frame_bundle().primary_frame.metadata.frame_num);

        if (result->keypoints.size() > 0) {
            RDX_LOG_DEBUG(this, __func__, "{}", "开始处理keypoints结果");
            if (result->is_matched_by_uid) { // 如果是基于x_group_id匹配的
                RDX_LOG_DEBUG(this, __func__, "{}", "基于x_group_id匹配keypoints");
                for (size_t i = 0; i < result->keypoints.size(); ++i) {
                    for (size_t j = 0; j < document->detections.size(); ++j) {
                        if (document->detections[j].x_uid == result->keypoints[i].x_group_uid) {
                            document->detections[j].keypoints = result->keypoints[i];
                            break;
                        }
                    }
                }
            } else { // 如果是按顺序保存的
                RDX_LOG_DEBUG(this, __func__, "{}", "按顺序保存keypoints");
                for (size_t i = 0; i < result->keypoints.size(); ++i) {
                    document->detections[output_model_result.source_data->get_detections_indices()[i]].keypoints = result->keypoints[i];
                }
            }
        }
        output_pipeline_source_data.set_document(*document);
        // create pipeline delivery request
        auto control_signal_code = output_model_result.control_signal_code;
        RDX_LOG_DEBUG(this, __func__, "{}", "开始创建delivery request");
        auto delivery_request = _create_delivery_request(output_pipeline_source_data, control_signal_code);
        // push to output port pipeline
        // this is used for logging
        auto msg_uuid = output_pipeline_source_data.get_uuid();

        // test log
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "control signal code: {}", int(control_signal_code));
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "document frame num: {}, control signal code: {}",
                     document->frame_bundle.primary_frame.metadata.frame_num, int(document->x_control.code));

        // get qos, controls how to retry and drop frames
        RDX_LOG_DEBUG(this, __func__, "{}", "开始获取QoS配置");
        auto &qos = runtime_config->pipeline_enqueue_policy;
        auto success = m_primary_output_port_pipeline->push_request(delivery_request, qos);

        if (success) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "[msg_uuid={}] success to push request",
                         boost::uuids::to_string(msg_uuid));

            if (init_config->create_debug_pub) {
                RDX_LOG_DEBUG(this, __func__, "{}", "开始创建debug图像");
                auto debug_image = _create_debug_image(*document);
                m_pub_model_enqueue.publish(debug_image, "");
            }
        } else {
            RDX_INFO_DEV(this, __func__,
                         "[msg_uuid={}] failed to push request",
                         boost::uuids::to_string(msg_uuid));
        }
    }
}

} // namespace redoxi_works
