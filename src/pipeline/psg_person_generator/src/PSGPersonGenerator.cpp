#include <psg_person_generator/PSGPersonGenerator.hpp>
#include <psg_common/msg_converter.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <json_struct/json_struct.h>
#include <PassengerFlow/utils/util_functions.h>

#define PRINT_THREAD_ID_IN_LOG (true)

namespace redoxi_works
{

struct PSGPersonGeneratorImpl {
    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    //! person extractor
    std::vector<PassengerFlow::PersonPtr> _extract_persons(const std::vector<PassengerFlow::DetectionPtr> &heads,
                                                           const std::vector<PassengerFlow::DetectionPtr> &body_dets,
                                                           const std::vector<PassengerFlow::DetectionPtr> &face_dets,
                                                           const std::vector<std::map<PassengerFlow::KeyPointSemanticType, PassengerFlow::Keypoint>>
                                                               &body_keypoints);

    // pull input, work on it and then send output
    using PullProcessSendHandler_t = redoxi_works::port_handlers::PullProcessSendHandler<PSGPersonGenerator::InputPort_t::MasterSpec_t,
                                                                                         PSGPersonGenerator::OutputPort_t::MasterSpec_t>;
    std::shared_ptr<PullProcessSendHandler_t> work_then_send_handler;
};

PSGPersonGenerator::PSGPersonGenerator(const std::string &name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(name, options)
{
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
}

int PSGPersonGenerator::_start()
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);

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
        auto interval = runtime_config->document_interval;
        m_impl->m_ros_time_token->start(interval);
    }

    return 0;
}

int PSGPersonGenerator::_stop()
{
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

int PSGPersonGenerator::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
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

    //! Create and initialize input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating input port");
    m_input_port = std::make_shared<InputPort_t>(this);
    m_input_port->init(init_config->input_port_config);

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

int PSGPersonGenerator::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
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

    RDX_INFO_DEV(this, __func__, false, "{}", "Creating document request handler");
    _create_document_request_handler(*runtime_config);

    return 0;
}

std::shared_ptr<PSGPersonGeneratorImpl> PSGPersonGenerator::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGPersonGeneratorImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}

PSGPersonGenerator::DeliveryRequest_t
    PSGPersonGenerator::_create_delivery_request(const OutputSourceData_t &source_data,
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

std::shared_ptr<PSGPersonGenerator::OutputPort_t>
    PSGPersonGenerator::_create_primary_output_port(const InitConfig_t &init_config)
{
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "create primary output port");
    auto port = std::make_shared<OutputPort_t>(this);
    auto &port_config = init_config.output_port_config;
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

int PSGPersonGenerator::_create_document_request_handler(const RuntimeConfig_t &runtime_config)
{
    using ProcessHandler_t = PSGPersonGeneratorImpl::PullProcessSendHandler_t;
    using InputDataTrait_t = PSGPersonGenerator::InputPort_t::ActionDataTrait_t;
    auto config = std::make_shared<ProcessHandler_t::InitConfig_t>();

    config->block_input_reading = runtime_config.enable_blocking_mode;
    config->block_resource_acquisition = runtime_config.enable_blocking_mode;

    auto enqueue_policy = runtime_config.frame_enqueue_policy;
    m_impl->work_then_send_handler = std::make_shared<ProcessHandler_t>();
    auto process_handler = m_impl->work_then_send_handler;
    process_handler->init(m_input_port.get(), m_primary_output_port.get(),
                          nullptr, config, enqueue_policy);

    process_handler->on_process_input_data =
        [this](ProcessHandler_t::OutputRequest_t *output_request,
               std::optional<ProcessHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
               ProcessHandler_t::InputActionResult_t *action_result,
               std::shared_ptr<const InputSourceData_t> source_data,
               ProcessHandler_t::ResourceToken_t &resource) {
            // process document, copy the document msg because the original one is const, cannot be modified
            psg_private_msgs::msg::PsgDocument document_msg;
            document_msg = source_data->m_goal->document;

            std::vector<PassengerFlow::DetectionPtr> v_heads, v_bodies, v_faces;
            std::vector<std::map<PassengerFlow::KeyPointSemanticType, PassengerFlow::Keypoint>> v_body_keypoints;
            // msg Detection {0: body, 1: head, 2: face}
            for (auto &msg_det : document_msg.detections) {
                if (msg_det.category == 0) {
                    v_bodies.push_back(std::make_shared<PassengerFlow::Detection>());
                    FlowRos2Pipeline::convert_msg_to_detection(msg_det, v_bodies.back());
                    v_body_keypoints.push_back(std::map<PassengerFlow::KeyPointSemanticType, PassengerFlow::Keypoint>());
                    FlowRos2Pipeline::convert_msg_to_keypoints(msg_det.keypoints, v_body_keypoints.back());
                } else if (msg_det.category == 1) {
                    v_heads.push_back(std::make_shared<PassengerFlow::Detection>());
                    FlowRos2Pipeline::convert_msg_to_detection(msg_det, v_heads.back());
                } else if (msg_det.category == 2) {
                    v_faces.push_back(std::make_shared<PassengerFlow::Detection>());
                    FlowRos2Pipeline::convert_msg_to_detection(msg_det, v_faces.back());
                }
            }

            // extract person
            auto v_persons = m_impl->_extract_persons(v_heads, v_bodies, v_faces, v_body_keypoints);

            // convert to msg
            for (auto &person : v_persons) {
                psg_private_msgs::msg::Person msg_person;
                msg_person.frame_metadata = document_msg.frame_bundle.primary_frame.metadata;
                FlowRos2Pipeline::convert_person_to_msg(person, document_msg.frame_bundle.primary_frame, msg_person);
                msg_person.x_uid = to_ros_uuid_msg(boost::uuids::random_generator()()); // 不加这个会导致tracker无法匹配
                document_msg.persons.push_back(msg_person);
            }

            // from input source data to output source data
            OutputSourceData_t output_source_data;
            output_source_data.set_document(document_msg);

            auto goal_handle = source_data->get_goal_handle_future().get();
            auto control_signal_code = InputDataTrait_t::get_control_signal_code(*source_data->get_goal());

            // create delivery request
            auto delivery_request = _create_delivery_request(output_source_data, control_signal_code);
            *output_request = delivery_request;

            auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
            if (init_config->create_debug_pub) {
                auto debug_image = _create_debug_image(document_msg);
                m_pub_task_enqueue.publish(debug_image, "");
            }

            // fill the action result, nothing to do
            (void)action_result;

            (void)output_enqueue_policy;
            (void)resource;
            return 0;
        };
    return 0;
}

int PSGPersonGenerator::_process_document_request()
{
    auto ret = m_impl->work_then_send_handler->process_and_send();
    if (ret == PSGPersonGeneratorImpl::PullProcessSendHandler_t::ProcessResult::Error) {
        RDX_INFO_DEV(this, __func__, false, "Failed to process image request, error code: {}", int(ret));
        return -1;
    } else if (ret == PSGPersonGeneratorImpl::PullProcessSendHandler_t::ProcessResult::NoData) {
        //! No data available, skipping
        return 0;
    } else if (ret == PSGPersonGeneratorImpl::PullProcessSendHandler_t::ProcessResult::Success) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Successfully processed image request");
        return 0;
    } else if (ret == PSGPersonGeneratorImpl::PullProcessSendHandler_t::ProcessResult::NoResourceToken) {
        //! No resource token, skipping
        return 0;
    } else if (ret == PSGPersonGeneratorImpl::PullProcessSendHandler_t::ProcessResult::FailedToSend) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Failed to send image request to downstream, do you have a downstream?");
        return 0;
    } else {
        RDX_RAISE_ERROR("[f={}] Unexpected process result: {}", __func__, int(ret));
        return -1;
    }
}

void PSGPersonGenerator::_step()
{
    if (m_input_port) {
        _process_document_request();
    }
}

// body_keypoints is same order as bodies, one to one, no missing
std::vector<PassengerFlow::PersonPtr> PSGPersonGeneratorImpl::_extract_persons(const std::vector<PassengerFlow::DetectionPtr> &heads,
                                                                               const std::vector<PassengerFlow::DetectionPtr> &bodies,
                                                                               const std::vector<PassengerFlow::DetectionPtr> &faces,
                                                                               const std::vector<std::map<PassengerFlow::KeyPointSemanticType, PassengerFlow::Keypoint>>
                                                                                   &body_keypoints)
{
    std::vector<std::pair<int, int>> matched_body_head, matched_head_face;
    std::vector<int> unmatched_body, unmatched_face, unmatched_head_without_face, unmatched_head_without_body;
    PassengerFlow::body_head_match(bodies, heads, &matched_body_head, &unmatched_body, &unmatched_head_without_body);
    PassengerFlow::head_face_match(heads, faces, &matched_head_face, &unmatched_head_without_face, &unmatched_face);

    std::vector<PassengerFlow::PersonPtr> output_persons;

    std::vector<int> used_head_index;
    // create body-head-face or body-head-null
    for (auto body_head_pair : matched_body_head) {
        PassengerFlow::PersonPtr temp_person = std::make_shared<PassengerFlow::Person>();
        PassengerFlow::DetectionPtr temp_face = nullptr;
        for (auto head_face_pair : matched_head_face) {
            if (body_head_pair.second == head_face_pair.first) {
                temp_face = faces[head_face_pair.second];
                break;
            }
        }
        temp_person->init(heads[body_head_pair.second],
                          temp_face,
                          bodies[body_head_pair.first]);
        // 如果有body,则添加对应的keypoints
        temp_person->set_keypoints(body_keypoints[body_head_pair.first]);
        output_persons.push_back(temp_person);
    }

    // create head-face
    for (auto head_face_pair : matched_head_face) {
        PassengerFlow::PersonPtr temp_person = std::make_shared<PassengerFlow::Person>();
        bool only_head_face = true;
        for (auto body_head_pair : matched_body_head) {
            if (body_head_pair.second == head_face_pair.first) {
                only_head_face = false;
                break;
            }
        }
        if (only_head_face) {
            temp_person->init(heads[head_face_pair.first], faces[head_face_pair.second], nullptr);
            output_persons.push_back(temp_person);
        }
    }

    // create only body
    for (auto unmatched_body_index : unmatched_body) {
        PassengerFlow::PersonPtr temp_person = std::make_shared<PassengerFlow::Person>();
        temp_person->init(nullptr, nullptr, bodies[unmatched_body_index]);
        // 如果有body,则添加对应的keypoints
        temp_person->set_keypoints(body_keypoints[unmatched_body_index]);
        output_persons.push_back(temp_person);
    }
    // create only head
    std::vector<int> unmatched_head;
    std::sort(unmatched_head_without_body.begin(), unmatched_head_without_body.end());
    std::sort(unmatched_head_without_face.begin(), unmatched_head_without_face.end());
    std::set_intersection(unmatched_head_without_body.begin(), unmatched_head_without_body.end(),
                          unmatched_head_without_face.begin(), unmatched_head_without_face.end(),
                          back_inserter(unmatched_head));
    for (auto unmatched_head_index : unmatched_head) {
        PassengerFlow::PersonPtr temp_person = std::make_shared<PassengerFlow::Person>();
        temp_person->init(heads[unmatched_head_index], nullptr, nullptr);
        output_persons.push_back(temp_person);
    }
    // create only face
    for (auto unmatched_face_index : unmatched_face) {
        PassengerFlow::PersonPtr temp_person = std::make_shared<PassengerFlow::Person>();
        temp_person->init(nullptr, faces[unmatched_face_index], nullptr);
        output_persons.push_back(temp_person);
    }


    return output_persons;
}

//! 将document中的raw image转换为带有检测框和body keypoints的debug image
sensor_msgs::msg::Image PSGPersonGenerator::_create_debug_image(const psg_private_msgs::msg::PsgDocument &document)
{
    //! 转换raw image到cv::Mat
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始转换raw image到cv::Mat", 0);
    cv::Mat cv_image;
    image_utils::FrameMediator fm(&document.frame_bundle.primary_frame);
    fm.to_cv_image_copy(cv_image);
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "成功转换raw image, 大小: {}x{}", cv_image.cols, cv_image.rows);


    //! 在图像上画person相关的框和keypoints
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始在图像上绘制person相关的框和keypoints, 共{}个person", document.persons.size());
    for (const auto &person : document.persons) {
        //! 随机生成颜色
        cv::Scalar color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);

        //! 获取body bbox坐标
        if (person.true_body.category == 0) {
            int x = static_cast<int>(person.true_body.bbox.x);
            int y = static_cast<int>(person.true_body.bbox.y);
            int width = static_cast<int>(person.true_body.bbox.width);
            int height = static_cast<int>(person.true_body.bbox.height);

            //! 画body bbox
            cv::rectangle(cv_image,
                          cv::Point(x, y),
                          cv::Point(x + width, y + height),
                          color, 2);
        }

        //! 画body keypoints
        const auto &keypoints = person.true_body.keypoints;

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
        if (person.true_head.category == 1) {
            int x = static_cast<int>(person.true_head.bbox.x);
            int y = static_cast<int>(person.true_head.bbox.y);
            int width = static_cast<int>(person.true_head.bbox.width);
            int height = static_cast<int>(person.true_head.bbox.height);

            //! 画head bbox
            cv::rectangle(cv_image,
                          cv::Point(x, y),
                          cv::Point(x + width, y + height),
                          color, 2);
        }

        //! 画face bbox
        if (person.true_face.category == 2) {
            int x = static_cast<int>(person.true_face.bbox.x);
            int y = static_cast<int>(person.true_face.bbox.y);
            int width = static_cast<int>(person.true_face.bbox.width);
            int height = static_cast<int>(person.true_face.bbox.height);

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
    image_utils::FrameMediator fm_cv_image(cv_image, "bgr8");
    fm_cv_image.to_image_msg(debug_image);
    RDX_LOG_DEBUG(this, __func__, PRINT_THREAD_ID_IN_LOG, "完成debug图像创建, 大小: {}x{}", debug_image.width, debug_image.height);
    return debug_image;
}

} // namespace redoxi_works
