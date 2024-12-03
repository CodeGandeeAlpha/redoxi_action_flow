#include <psg_tracker/Pipeline.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_samples_lib/random_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <json_struct/json_struct.h>
#include <tbb/concurrent_queue.h>
#include <RedoxiTrack/RedoxiTrack.h>
#include <psg_common/psg_common.hpp>
// #include <tbb/task_group.h>

#define PRINT_THREAD_ID_IN_LOG (true)

namespace redoxi_works
{
// shared_ptr is used to indicate whether the result is available
// 如果我在request创建的时候就把promise和future绑定在一起，那么当request发送失败的时候我也可以把结果设置为nullptr来表示失败
using ModelResultPromise = std::promise<std::shared_ptr<PSGTrackerPipelineNode::OutputModelResult_t>>;
using ModelResultFuture = std::shared_future<std::shared_ptr<PSGTrackerPipelineNode::OutputModelResult_t>>;

struct PSGTrackerPipelineImpl {
    struct OutputModelResult {
        std::shared_ptr<ModelResultPromise> promise;
        ModelResultFuture future;
        std::shared_ptr<PSGTrackerPipelineNode::OutputSourceDataModel_t> source_data;
    };

    using ArrayUUID = std::array<uint8_t, 16>;

    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    //! buffer the model result
    tbb::concurrent_bounded_queue<OutputModelResult> m_model_result_buffer;

    //! task group for model result
    tbb::task_group m_model_result_task_group;

    //! 记录document的异步字典
    using DocumentMap_t = boost::synchronized_value<std::map<int, std::shared_ptr<psg_private_msgs::msg::PsgDocument>>>;
    DocumentMap_t m_document_map;

    //! 记录person的异步字典
    using PersonMap_t = boost::synchronized_value<std::map<ArrayUUID, std::shared_ptr<psg_private_msgs::msg::Person>>>;
    PersonMap_t m_person_map;

    //! 记录trajectory的异步字典
    std::map<int, std::vector<ArrayUUID>> m_closed_trajectory_map; // indexed by track id
};

PSGTrackerPipelineNode::PSGTrackerPipelineNode(const std::string &name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(name, options)
{
}

PSGTrackerPipelineNode::~PSGTrackerPipelineNode()
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

int PSGTrackerPipelineNode::_start()
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

int PSGTrackerPipelineNode::_stop()
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

void PSGTrackerPipelineNode::set_publish_to_debug_topic(bool enable)
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

bool PSGTrackerPipelineNode::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int PSGTrackerPipelineNode::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
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

int PSGTrackerPipelineNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
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

    return 0;
}

std::shared_ptr<PSGTrackerPipelineImpl> PSGTrackerPipelineNode::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGTrackerPipelineImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

PSGTrackerPipelineNode::DeliveryRequestPipeline_t
    PSGTrackerPipelineNode::_create_delivery_request(const OutputSourceDataPipeline_t &source_data)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    //! Create delivery request
    DeliveryRequestPipeline_t req;
    req.set_source_data(source_data);
    if (runtime_config->pipeline_request_policy.has_value()) {
        req.set_delivery_policy(*runtime_config->pipeline_request_policy);
    }

    return req;
}

PSGTrackerPipelineNode::DeliveryRequestModel_t
    PSGTrackerPipelineNode::_create_delivery_request(const OutputSourceDataModel_t &source_data)
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    //! Create delivery request
    DeliveryRequestModel_t req;
    req.set_source_data(source_data);
    if (runtime_config->model_request_policy.has_value()) {
        req.set_delivery_policy(*runtime_config->model_request_policy);
    }
    return req;
}

std::shared_ptr<PSGTrackerPipelineNode::OutputPortPipeline_t>
    PSGTrackerPipelineNode::_create_primary_output_port_pipeline(const InitConfig_t &init_config)
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

std::shared_ptr<PSGTrackerPipelineNode::OutputPortModel_t>
    PSGTrackerPipelineNode::_create_primary_output_port_model(const InitConfig_t &init_config)
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

void PSGTrackerPipelineNode::_step()
{
    // 从input port pipeline获取数据，创建delivery request，并推送到output port model,
    // 从input port model获取数据，放到detections buffer中去
    if (get_status() != NodeStatusCode::STARTED) {
        return;
    }

    if (m_impl->m_ros_time_token->try_pop_token()) {
        auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);

        std::shared_ptr<InputSourceData_t> document_data;
        if (runtime_config->enable_blocking_mode) {
            // wait until there is data available
            document_data = m_input_port->pop_source_data();
        } else {
            // try to get data without waiting
            document_data = m_input_port->try_pop_source_data();
        }

        if (!document_data) {
            return;
        }

        // 将document数据放入document map中
        m_impl->m_document_map.synchronize()->insert({document_data->get_goal()->document.frame.metadata.frame_num,
                                                      std::make_shared<psg_private_msgs::msg::PsgDocument>(document_data->get_goal()->document)});

        // 将person数据放入person map中
        auto lock_ptr_person_map = m_impl->m_person_map.synchronize();
        for (const auto &person : document_data->get_goal()->document.persons) {
            lock_ptr_person_map->insert({person.x_uid.uuid, std::make_shared<psg_private_msgs::msg::Person>(person)});
        }

        // 创建delivery request，并推送到output port model
        // from input source data to output source data
        OutputSourceDataModel_t output_model_source_data;
        output_model_source_data.set_frame(document_data->get_goal()->document.frame);
        output_model_source_data.set_persons(document_data->get_goal()->document.persons);

        // create delivery request
        auto delivery_request = _create_delivery_request(output_model_source_data);

        // this is used for logging
        auto msg_uuid = output_model_source_data.get_uuid();

        // get qos, controls how to retry and drop frames
        auto &qos = runtime_config->model_enqueue_policy;
        auto success = m_primary_output_port_model->push_request(delivery_request, qos);

        if (success) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "[msg_uuid={}] success to push request to model, frame_num={}",
                         boost::uuids::to_string(msg_uuid),
                         document_data->get_goal()->document.frame.metadata.frame_num);
        } else {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "[msg_uuid={}] failed to push request to model, frame_num={}",
                         boost::uuids::to_string(msg_uuid),
                         document_data->get_goal()->document.frame.metadata.frame_num);
        }

        // FIXME: debug only
        // wait for all requests to be processed, not necessary
        m_primary_output_port_model->wait_for_all_requests();
    }
}

// for test
// void PSGTrackerPipelineNode::_step()
// {
//     // 从input port pipeline获取数据，创建delivery request，并推送到output port model,
//     // 从input port model获取数据，放到detections buffer中去
//     if (get_status() != NodeStatusCode::STARTED) {
//         return;
//     }

//     psg_private_msgs::msg::PsgDocument doc_msg;
//     _generate_source_data(doc_msg, m_frame_number);

//     // 将document数据放入document map中
//     m_impl->m_document_map.synchronize()->insert({doc_msg.frame.metadata.frame_num,
//                                                   std::make_shared<psg_private_msgs::msg::PsgDocument>(doc_msg)});

//     // 将person数据放入person map中
//     auto lock_ptr_person_map = m_impl->m_person_map.synchronize();
//     for (const auto &person : doc_msg.persons) {
//         lock_ptr_person_map->insert({person.x_uid.uuid, std::make_shared<psg_private_msgs::msg::Person>(person)});
//     }

//     // 创建delivery request，并推送到output port model
//     // from input source data to output source data
//     OutputSourceDataModel_t output_model_source_data;
//     output_model_source_data.set_frame(doc_msg.frame);
//     output_model_source_data.set_persons(doc_msg.persons);

//     // create delivery request
//     auto delivery_request = _create_delivery_request(output_model_source_data);

//     // this is used for logging
//     auto msg_uuid = output_model_source_data.get_uuid();

//     // get qos, controls how to retry and drop frames
//     auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
//     auto &qos = runtime_config->model_enqueue_policy;
//     auto success = m_primary_output_port_model->push_request(delivery_request, qos);

//     if (success) {
//         RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
//                      "[msg_uuid={}] success to push request",
//                      boost::uuids::to_string(msg_uuid));
//     } else {
//         RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
//                      "[msg_uuid={}] failed to push request",
//                      boost::uuids::to_string(msg_uuid));
//     }

//     // FIXME: debug only
//     // wait for all requests to be processed, not necessary
//     m_primary_output_port_model->wait_for_all_requests();
// }

void PSGTrackerPipelineNode::_generate_source_data(psg_private_msgs::msg::PsgDocument &document, std::atomic<int64_t> &frame_number)
{
    //! Generate a random frame with the UUID text
    // cv::Mat random_frame = cv::imread("data/ori_img.jpg");
    cv::Mat random_frame;
    auto frame_text = fmt::format("Frame Number: {}", frame_number.load());
    random_image_with_text(random_frame, cv::Size(1920, 1080), frame_text);


    // generate solid persons
    psg_private_msgs::msg::Person person;
    person.x_uid = to_ros_uuid_msg(boost::uuids::random_generator()());
    person.track_id = 1;
    person.body.bbox.x = 100;
    person.body.bbox.y = 100;
    person.body.bbox.width = 100;
    person.body.bbox.height = 100;
    person.body.category = 0;
    person.body.confidence = 0.9;
    document.persons.push_back(person);


    // convert image to ROS message
    cv_bridge::CvImage cv_bridge_image;
    cv_bridge_image.image = random_frame;
    cv_bridge_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_bridge_image.toImageMsg(document.frame.raw_image);
    document.frame.metadata.frame_num = frame_number;

    frame_number++;
}

int PSGTrackerPipelineNode::_on_deliver_to_downstream_finish(TargetDataModel_t &target_data,
                                                             SendResultModel_t &result,
                                                             const DeliveryRequestModel_t &request,
                                                             const DownstreamModel_t &ds)
{
    (void)target_data;

    //! 1. 创建modelresult
    RDX_INFO_DEV(this, __func__, false, "{}", "开始创建model result");
    PSGTrackerPipelineImpl::OutputModelResult output_model_result;

    //! 2. 绑定promise和future
    RDX_INFO_DEV(this, __func__, false, "{}", "开始绑定promise和future");
    output_model_result.promise = std::make_shared<ModelResultPromise>();
    output_model_result.future = output_model_result.promise->get_future().share();
    output_model_result.source_data = std::make_shared<OutputSourceDataModel_t>(request.get_source_data());

    //! 3. 将output_model_result推送到buffer中
    RDX_INFO_DEV(this, __func__, false, "{}", "开始将output_model_result推送到buffer中");
    m_impl->m_model_result_buffer.push(output_model_result);

    //! 4. 创建task 在tbb run中将结果写入promise
    RDX_INFO_DEV(this, __func__, false, "{}", "开始创建task并运行");
    auto promise = output_model_result.promise;
    //! 通过值捕获需要的数据
    auto ds_copy = ds;
    auto result_copy = result;
    m_impl->m_model_result_task_group.run([ds = ds_copy,
                                           result = result_copy,
                                           promise,
                                           this]() {
        (void)this;
        RDX_INFO_DEV(this, __func__, false, "{}", "开始获取goal handle");
        auto goal_handle = result.goal_handle_future.get();
        if (goal_handle) {
            RDX_INFO_DEV(this, __func__, false, "{}", "开始获取action result并写入promise");
            auto action_result = ds.get_action_client()->async_get_result(goal_handle).get().result;
            // 将action result写入promise
            auto final_output_model_result = std::make_shared<PSGTrackerPipelineNode::OutputModelResult_t>();
            final_output_model_result->track_targets = action_result->track_targets;
            final_output_model_result->x_return = action_result->x_return;
            promise->set_value(final_output_model_result);
        } else {
            RDX_INFO_DEV(this, __func__, false, "{}", "goal handle为空,写入空promise");
            promise->set_value(nullptr);
        }
    });

    return 0;
}

//! 将document中的raw image转换为带有track targets的debug image
sensor_msgs::msg::Image PSGTrackerPipelineNode::_create_debug_image(const psg_private_msgs::msg::PsgDocument &document)
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

    //! 在图像上画关键点和骨架连接
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始在图像上绘制关键点, 共{}个检测结果", document.detections.size());
    for (const auto &person : document.persons) {
        //! 根据person的track_id设置颜色
        cv::Scalar color = get_color(person.track_id);

        //! 获取body bbox坐标
        if (person.body.category == 0) {
            int x = static_cast<int>(person.body.bbox.x);
            int y = static_cast<int>(person.body.bbox.y);
            int width = static_cast<int>(person.body.bbox.width);
            int height = static_cast<int>(person.body.bbox.height);

            //! 画body bbox
            cv::rectangle(cv_image,
                          cv::Point(x, y),
                          cv::Point(x + width, y + height),
                          color, 2);
        }

        //! 画body keypoints
        const auto &keypoints = person.body.keypoints;

        //! 在访问数组或指针前添加检查
        if (!keypoints.keypoints_2.empty() && !keypoints.confidence.empty()) {
            //! 画出17个关键点
            for (size_t i = 0; i < keypoints.keypoints_2.size(); i++) {
                if (keypoints.confidence[i] > 0.3) { // 只画置信度大于0.3的点
                    //! 记录关键点的位置和置信度
                    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                                  "绘制关键点[{}] - 位置:[{}, {}], 置信度:{}",
                                  i, keypoints.keypoints_2[i].x, keypoints.keypoints_2[i].y, keypoints.confidence[i]);
                    cv::circle(cv_image,
                               cv::Point(keypoints.keypoints_2[i].x, keypoints.keypoints_2[i].y),
                               3, color, -1);
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
                             color, 2);
                }
            }
        }

        //! 画head bbox
        if (person.head.category == 1) {
            int x = static_cast<int>(person.head.bbox.x);
            int y = static_cast<int>(person.head.bbox.y);
            int width = static_cast<int>(person.head.bbox.width);
            int height = static_cast<int>(person.head.bbox.height);

            //! 画head bbox
            cv::rectangle(cv_image,
                          cv::Point(x, y),
                          cv::Point(x + width, y + height),
                          color, 2);
        }

        //! 画face bbox
        if (person.face.category == 2) {
            int x = static_cast<int>(person.face.bbox.x);
            int y = static_cast<int>(person.face.bbox.y);
            int width = static_cast<int>(person.face.bbox.width);
            int height = static_cast<int>(person.face.bbox.height);

            //! 画face bbox
            cv::rectangle(cv_image,
                          cv::Point(x, y),
                          cv::Point(x + width, y + height),
                          color, 2);
        }
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

void PSGTrackerPipelineNode::_get_model_result()
{
    //! 1. 从buffer中取出model result, 如果buffer为空，则等待
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始从buffer中取出model result", 0);
    PSGTrackerPipelineImpl::OutputModelResult output_model_result;
    m_impl->m_model_result_buffer.pop(output_model_result);
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "成功从buffer中取出model result", 0);

    //! 2. 等待结果
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始等待model result future", 0);
    auto result = output_model_result.future.get();
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "got model result, size: {}", result->track_targets.size());

    //! 3. 如果结果不为空，则构造output source data，并推送到output port pipeline
    if (result) {
        auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
        auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);

        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始构造output source data", 0);
        // create output source data
        OutputSourceDataPipeline_t output_pipeline_source_data;
        // 根据frame_number获取document
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始从document map中获取document", 0);
        auto document = m_impl->m_document_map.synchronize()->at(output_model_result.source_data->get_frame().metadata.frame_num);
        // 删掉字典中的document
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始从document map中删除document", 0);
        m_impl->m_document_map.synchronize()->erase(output_model_result.source_data->get_frame().metadata.frame_num);

        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始处理track_targets结果", 0);
        for (const auto &track_target : result->track_targets) {
            RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                          "处理track_target - track_id:{}, track_status:{}, uuid:{}",
                          track_target.track_id,
                          track_target.track_status.value,
                          boost::uuids::to_string(to_boost_uuid(track_target.x_group_uid.uuid)));

            // 1. 将track_targets中的track_id和person_id进行匹配，并赋予跟踪的id
            {
                auto lock_ptr_person_map = m_impl->m_person_map.synchronize();
                if (lock_ptr_person_map->find(track_target.x_group_uid.uuid) != lock_ptr_person_map->end()) {
                    auto &person = (*lock_ptr_person_map)[track_target.x_group_uid.uuid];
                    person->track_id = track_target.track_id;
                    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                                  "更新person track_id - uuid:{}, track_id:{}",
                                  boost::uuids::to_string(to_boost_uuid(track_target.x_group_uid.uuid)),
                                  track_target.track_id);
                }
            }

            // 2. 将track_target中的track_id写入document中的persons
            for (auto &person : document->persons) {
                if (person.x_uid == track_target.x_group_uid) {
                    person.track_id = track_target.track_id;
                    break;
                }
            }

            // 3. 收集closed trajectory，并写入document中的trajectories
            // if track_target is new, create a new trajectory
            if (track_target.track_status.value == redoxi_public_msgs::msg::TrackObjectStatus::NEW) {
                m_impl->m_closed_trajectory_map[track_target.track_id] = std::vector<PSGTrackerPipelineImpl::ArrayUUID>();
                m_impl->m_closed_trajectory_map[track_target.track_id].push_back(track_target.x_group_uid.uuid);
                RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                              "创建新轨迹 - track_id:{}, uuid:{}",
                              track_target.track_id,
                              boost::uuids::to_string(to_boost_uuid(track_target.x_group_uid.uuid)));
            }
            // if track_target is open, add it to trajectory
            else if (track_target.track_status.value == redoxi_public_msgs::msg::TrackObjectStatus::OPEN) {
                m_impl->m_closed_trajectory_map[track_target.track_id].push_back(track_target.x_group_uid.uuid);
                RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                              "添加到现有轨迹 - track_id:{}, uuid:{}",
                              track_target.track_id,
                              boost::uuids::to_string(to_boost_uuid(track_target.x_group_uid.uuid)));
            }
            // if track_target is close, get trajectory and remove it from buffer
            else if (track_target.track_status.value == redoxi_public_msgs::msg::TrackObjectStatus::CLOSE) {
                // get closed trajectory uuids
                auto closed_trajectory_uuids = m_impl->m_closed_trajectory_map[track_target.track_id];
                // remove closed trajectory from buffer
                m_impl->m_closed_trajectory_map.erase(track_target.track_id);
                // get closed trajectory
                psg_private_msgs::msg::PersonTrajectory closed_trajectory;
                closed_trajectory.track_id = track_target.track_id;
                {
                    auto lock_ptr_person_map = m_impl->m_person_map.synchronize();
                    for (auto &uuid : closed_trajectory_uuids) {
                        closed_trajectory.persons.push_back(*(*lock_ptr_person_map)[uuid]);
                        if (lock_ptr_person_map->find(uuid) != lock_ptr_person_map->end()) {
                            lock_ptr_person_map->erase(uuid);
                        }
                    }
                }
                // put closed trajectory to document
                document->trajectories.push_back(closed_trajectory);
                RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG,
                              "关闭轨迹 - track_id:{}, 轨迹长度:{}",
                              track_target.track_id,
                              closed_trajectory.persons.size());
            } else
                continue;
        }
        output_pipeline_source_data.set_document(*document);

        // create pipeline delivery request
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始创建delivery request", 0);
        auto delivery_request = _create_delivery_request(output_pipeline_source_data);
        // push to output port pipeline
        // this is used for logging
        auto msg_uuid = output_pipeline_source_data.get_uuid();

        // get qos, controls how to retry and drop frames
        RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始获取QoS配置", 0);
        auto &qos = runtime_config->pipeline_enqueue_policy;
        auto success = m_primary_output_port_pipeline->push_request(delivery_request, qos);

        if (success) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "[msg_uuid={}] success to push request",
                         boost::uuids::to_string(msg_uuid));

            if (init_config->create_debug_pub) {
                RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始创建debug图像", 0);
                auto debug_image = _create_debug_image(*document);
                m_pub_model_enqueue.publish(debug_image, "");
            }
        } else {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
                         "[msg_uuid={}] failed to push request",
                         boost::uuids::to_string(msg_uuid));
        }
    }
}

} // namespace redoxi_works
