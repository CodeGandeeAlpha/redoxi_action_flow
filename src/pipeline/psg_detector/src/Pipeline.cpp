#include <psg_detector/PipelineTypes.hpp>
#include <psg_detector/Pipeline.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_samples_lib/random_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <json_struct/json_struct.h>
#include <tbb/concurrent_queue.h>

// #include <tbb/task_group.h>

#define PRINT_THREAD_ID_IN_LOG (true)

namespace redoxi_works
{
// shared_ptr is used to indicate whether the result is available
// 如果我在request创建的时候就把promise和future绑定在一起，那么当request发送失败的时候我也可以把结果设置为nullptr来表示失败
using ModelResultPromise = std::promise<std::shared_ptr<PSGDetectorNode::OutputModelResult_t>>;
using ModelResultFuture = std::shared_future<std::shared_ptr<PSGDetectorNode::OutputModelResult_t>>;

struct PSGDetectorImpl {
    struct OutputModelResult {
        std::shared_ptr<ModelResultPromise> promise;
        ModelResultFuture future;
        std::shared_ptr<PSGDetectorNode::OutputSourceDataModel_t> source_data;
    };

    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    //! buffer the model result
    tbb::concurrent_bounded_queue<OutputModelResult> m_model_result_buffer;

    //! task group for model result
    tbb::task_group m_model_result_task_group;
};

PSGDetectorNode::PSGDetectorNode(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options)
{
    _declare_all_parameters();
}

// void test_tbb()
// {
//     tbb::task_group tg;
//     tg.run([]{
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     });
//     tg.wait();
// }

PSGDetectorNode::~PSGDetectorNode()
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

    // stop step thread
    m_step_running = false;
    if (m_step_thread != nullptr && m_step_thread->joinable()) {
        m_step_thread->join();
    }

    // stop get model result thread
    m_get_model_result_thread_running = false;
    if (m_get_model_result_thread != nullptr && m_get_model_result_thread->joinable()) {
        m_get_model_result_thread->join();
    }

    // destroy impl task group
    m_impl->m_model_result_task_group.wait();
}

int PSGDetectorNode::start()
{
    //! Can only start in STOPPED status
    if (m_status_code != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[{}] status must be in STOPPED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

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
        auto interval = m_runtime_config->document_interval;
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

    //! start get model result thread
    m_get_model_result_thread_running = true;
    m_get_model_result_thread = std::make_shared<std::thread>([this]() {
        while (m_status_code == NodeStatusCode::STARTED && rclcpp::ok() && m_get_model_result_thread_running) {
            _get_model_result();
        }
    });

    return 0;
}

int PSGDetectorNode::stop()
{
    //! Can only stop in STARTED status
    if (m_status_code != NodeStatusCode::STARTED) {
        RDX_RAISE_ERROR("[{}] status must be in STARTED, got {}", __func__, NodeStatusCodeToString(m_status_code));
        return -1;
    }

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

    //! Stop get model result thread
    m_get_model_result_thread_running = false;
    if (m_get_model_result_thread != nullptr && m_get_model_result_thread->joinable()) {
        m_get_model_result_thread->join();
    }

    //! destroy impl task group
    m_impl->m_model_result_task_group.wait();

    return 0;
}

void PSGDetectorNode::set_publish_to_debug_topic(bool enable)
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

bool PSGDetectorNode::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int PSGDetectorNode::init(std::shared_ptr<InitConfig_t> config,
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

int PSGDetectorNode::update_init_config(std::shared_ptr<InitConfig_t> config)
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
    auto primary_output_port_pipeline = _create_primary_output_port_pipeline();
    if (!primary_output_port_pipeline) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port_pipeline = primary_output_port_pipeline;

    auto primary_output_port_model = _create_primary_output_port_model();
    if (!primary_output_port_model) {
        RDX_RAISE_ERROR("[{}] Failed to create primary output port", __func__);
    }
    m_primary_output_port_model = primary_output_port_model;

    //! Initialize debug publishers
    if (m_init_config->create_debug_pub) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                     "initialize debug publishers, pipeline enqueue topic={}, pipeline drop topic={}, model enqueue topic={}, model drop topic={}",
                     m_init_config->debug_pub_pipeline_enqueue_name,
                     m_init_config->debug_pub_pipeline_drop_name,
                     m_init_config->debug_pub_model_enqueue_name,
                     m_init_config->debug_pub_model_drop_name);
        auto debug_qos = DefaultParams::DebugPublisherQoS;
        m_pub_pipeline_enqueue.init(this, m_init_config->debug_pub_pipeline_enqueue_name, debug_qos);
        m_pub_pipeline_drop.init(this, m_init_config->debug_pub_pipeline_drop_name, debug_qos);
        m_pub_model_enqueue.init(this, m_init_config->debug_pub_model_enqueue_name, debug_qos);
        m_pub_model_drop.init(this, m_init_config->debug_pub_model_drop_name, debug_qos);
    }

    return 0;
}

int PSGDetectorNode::update_runtime_config(std::shared_ptr<RuntimeConfig_t> config)
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
    m_primary_output_port_pipeline->set_callback_on_request_enqueued([](DeliveryRequestPipeline_t &request) {
        // do nothing
    });
    m_primary_output_port_model->set_callback_on_request_enqueued([](DeliveryRequestModel_t &request) {
        // do nothing
    });

    //! set publish to debug topic
    set_publish_to_debug_topic(config->publish_to_debug_topic);

    return 0;
}

std::shared_ptr<PSGDetectorImpl> PSGDetectorNode::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGDetectorImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

void PSGDetectorNode::_set_status_code(int status_code)
{
    m_status_code = status_code;
}

PSGDetectorNode::DeliveryRequestPipeline_t
    PSGDetectorNode::_create_delivery_request(const OutputSourceDataPipeline_t &source_data)
{
    //! Create delivery request
    DeliveryRequestPipeline_t req;
    req.set_source_data(source_data);
    if (m_runtime_config->pipeline_request_policy.has_value()) {
        req.set_delivery_policy(*m_runtime_config->pipeline_request_policy);
    }

    return req;
}

PSGDetectorNode::DeliveryRequestModel_t
    PSGDetectorNode::_create_delivery_request(const OutputSourceDataModel_t &source_data)
{
    //! Create delivery request
    DeliveryRequestModel_t req;
    req.set_source_data(source_data);
    if (m_runtime_config->model_request_policy.has_value()) {
        req.set_delivery_policy(*m_runtime_config->model_request_policy);
    }
    return req;
}

std::shared_ptr<PSGDetectorNode::OutputPortPipeline_t>
    PSGDetectorNode::_create_primary_output_port_pipeline()
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port pipeline");
    auto port = std::make_shared<OutputPortPipeline_t>(this);
    auto &port_config = m_init_config->output_port_pipeline_config;
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

std::shared_ptr<PSGDetectorNode::OutputPortModel_t>
    PSGDetectorNode::_create_primary_output_port_model()
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port model");
    auto port = std::make_shared<OutputPortModel_t>(this);
    auto &port_config = m_init_config->output_port_model_config;
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

int PSGDetectorNode::_declare_all_parameters()
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

void PSGDetectorNode::_step()
{
    // 从input port pipeline获取数据，创建delivery request，并推送到output port model,
    // 从input port model获取数据，放到detections buffer中去
    if (m_status_code != NodeStatusCode::STARTED) {
        return;
    }

    if (m_impl->m_ros_time_token->try_pop_token()) {
        std::shared_ptr<InputSourceData_t> document_data;
        if (m_init_config->enable_blocking_mode) {
            // wait until there is data available
            document_data = m_input_port->pop_source_data();
        } else {
            // try to get data without waiting
            document_data = m_input_port->try_pop_source_data();
        }

        if (!document_data) {
            return;
        }

        // 创建delivery request，并推送到output port model
        // from input source data to output source data
        OutputSourceDataModel_t output_model_source_data;
        output_model_source_data.set_document(document_data->get_goal()->document);

        // create delivery request
        auto delivery_request = _create_delivery_request(output_model_source_data);

        // this is used for logging
        auto msg_uuid = output_model_source_data.get_uuid();

        // get qos, controls how to retry and drop frames
        auto &qos = m_runtime_config->model_enqueue_policy;
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
            while (!m_primary_output_port_model->try_push_request(delivery_request)) {
                std::this_thread::sleep_for(interval_between_attempts);
            }
            success = true;
        } else if (drop_frame_strategy == DropStrategy::DropAsNeeded) {
            // Try up to max attempts if dropping is allowed
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                if (m_primary_output_port_model->try_push_request(delivery_request)) {
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
        m_primary_output_port_model->wait_for_all_requests();
    }
}

int PSGDetectorNode::_on_deliver_to_downstream_finish(TargetDataModel_t &target_data,
                                                      SendResultModel_t &result,
                                                      const DeliveryRequestModel_t &request,
                                                      const DownstreamModel_t &ds)
{
    // 1. 创建modelresult
    PSGDetectorImpl::OutputModelResult output_model_result;

    // 2. 绑定promise和future
    output_model_result.promise = std::make_shared<ModelResultPromise>();
    output_model_result.future = output_model_result.promise->get_future().share();
    output_model_result.source_data = std::make_shared<OutputSourceDataModel_t>(request.get_source_data());

    // 3. 将output_model_result推送到buffer中
    m_impl->m_model_result_buffer.push(output_model_result);

    // 4. 创建task 在tbb run中将结果写入promise
    auto promise = output_model_result.promise;
    //! 通过值捕获需要的数据
    auto ds_copy = ds;
    auto result_copy = result;
    m_impl->m_model_result_task_group.run([ds = std::move(ds_copy),
                                           result = std::move(result_copy),
                                           promise]() {
        auto goal_handle = result.goal_handle_future.get();
        if (goal_handle) {
            auto action_result = ds.get_action_client()->async_get_result(goal_handle).get().result;
            // 将action result写入promise
            auto output_model_result = std::make_shared<PSGDetectorNode::OutputModelResult_t>();
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
sensor_msgs::msg::Image PSGDetectorNode::_create_debug_image(const psg_private_msgs::msg::PsgDocument &document)
{
    //! 转换raw image到cv::Mat
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始转换raw image到cv::Mat", 0);
    cv::Mat cv_image;
    try {
        cv_image = cv_bridge::toCvCopy(document.frame.raw_image, sensor_msgs::image_encodings::BGR8)->image;
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "成功转换raw image, 大小: {}x{}", cv_image.cols, cv_image.rows);
    } catch (const cv_bridge::Exception &e) {
        RDX_LOG_ERROR(this, __func__, PRINT_THREAD_ID_IN_LOG, "cv_bridge转换失败: {}", e.what());
        return sensor_msgs::msg::Image(); // 返回空图像
    }

    //! 为不同类别设置不同颜色
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "设置类别颜色映射", 0);
    std::map<int, cv::Scalar> class_colors = {
        {0, cv::Scalar(0, 255, 0)}, // 人-绿色
        {1, cv::Scalar(0, 0, 255)}, // 头-红色
        {2, cv::Scalar(255, 0, 0)}, // 脸-蓝色
    };

    //! 在图像上画bbox
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始在图像上绘制检测框, 共{}个检测结果", document.detections.detections.size());
    for (const auto &detection : document.detections.detections) {
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
    debug_image.header = document.frame.raw_image.header;
    debug_image.height = cv_image.rows;
    debug_image.width = cv_image.cols;
    debug_image.encoding = "bgr8";
    debug_image.is_bigendian = false;
    debug_image.step = cv_image.cols * 3;
    debug_image.data.assign(cv_image.data, cv_image.data + cv_image.total() * cv_image.elemSize());

    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "完成debug图像创建, 大小: {}x{}", debug_image.width, debug_image.height);
    return debug_image;
}

void PSGDetectorNode::_get_model_result()
{
    // 1. 从buffer中取出model result, 如果buffer为空，则等待
    PSGDetectorImpl::OutputModelResult output_model_result;
    m_impl->m_model_result_buffer.pop(output_model_result);
    // 2. 等待结果
    auto result = output_model_result.future.get();
    // 3. 如果结果不为空，则构造output source data，并推送到output port pipeline
    if (result) {
        // create output source data
        OutputSourceDataPipeline_t output_pipeline_source_data;
        auto document = output_model_result.source_data->get_document();
        document.detections = result->detections;
        output_pipeline_source_data.set_document(document);
        // create pipeline delivery request
        auto delivery_request = _create_delivery_request(output_pipeline_source_data);
        // push to output port pipeline
        // this is used for logging
        auto msg_uuid = output_pipeline_source_data.get_uuid();

        // get qos, controls how to retry and drop frames
        auto &qos = m_runtime_config->model_enqueue_policy;
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
            while (!m_primary_output_port_pipeline->try_push_request(delivery_request)) {
                std::this_thread::sleep_for(interval_between_attempts);
            }
            success = true;
        } else if (drop_frame_strategy == DropStrategy::DropAsNeeded) {
            // Try up to max attempts if dropping is allowed
            for (int attempt = 0; attempt < max_attempts; ++attempt) {
                if (m_primary_output_port_pipeline->try_push_request(delivery_request)) {
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

            std::string det_str;
            for (const auto &det : result->detections.detections) {
                det_str += fmt::format("bbox: [{}, {}, {}, {}] ",
                                       det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height);
            }
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "[msg_uuid={}] got detection result: {}",
                         boost::uuids::to_string(output_model_result.source_data->get_uuid()), det_str);

            if (m_init_config->create_debug_pub) {
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

} // namespace redoxi_works
