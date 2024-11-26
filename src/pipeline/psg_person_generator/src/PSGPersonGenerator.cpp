#include <psg_person_generator/PSGPersonGenerator.hpp>
#include <psg_common/msg_converter.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <cv_bridge/cv_bridge.hpp>
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
    PSGPersonGenerator::_create_delivery_request(const OutputSourceData_t &source_data)
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

void PSGPersonGenerator::_step()
{
    if (get_status() != NodeStatusCode::STARTED) {
        return;
    }

    // get input document msg
    if (m_impl->m_ros_time_token->try_pop_token()) {
        auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);

        std::shared_ptr<InputSourceData_t> document_in_data;
        if (runtime_config->enable_blocking_mode) {
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
            msg_person.frame_metadata = document_msg.frame.metadata;
            FlowRos2Pipeline::convert_person_to_msg(person, document_msg.frame, msg_person);
            document_msg.persons.push_back(msg_person);
        }

        // create tasks
        // document_msg.persons = persons_msg;

        // from input source data to output source data
        OutputSourceData_t output_source_data;
        output_source_data.set_document(document_msg);

        // create delivery request
        auto delivery_request = _create_delivery_request(output_source_data);

        // this is used for logging
        auto msg_uuid = output_source_data.get_uuid();

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

} // namespace redoxi_works
