#include <psg_document_sink/psg_document_sink.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <dynmsg/message_reading.hpp>
#include <dynmsg/msg_parser.hpp>
#include <dynmsg/typesupport.hpp>
#include <dynmsg/yaml_utils.hpp>
#include <fstream>

namespace redoxi_works
{

PSGDocumentSinkInitConfig::PSGDocumentSinkInitConfig()
{
    input_port_config->set_action_name("in/action");
    input_port_config->set_buffer_capacity(1);
}

PSGDocumentSink::PSGDocumentSink(const std::string &node_name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(node_name, options)
{
}

PSGDocumentSink::~PSGDocumentSink()
{
}

int PSGDocumentSink::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "updating init config");

    //! Step 1: Cast config to InitConfig_t
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(config);
    if (!init_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast init config to FrameRelayNodeInitConfig", __func__);
    }

    //! Step 2: Log init config as JSON for debugging
    // {
    //     RDX_INFO_DEV(this, __func__, false, "{}", "Parsing init config as JSON");
    //     auto init_config_json = JS::serializeStruct(*init_config);
    //     RDX_INFO_DEV(this, __func__, false, "InitConfig JSON: {}", init_config_json);
    // }

    //! Step 3: Create and initialize input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating input port");
    m_input_port = std::make_shared<InputPort_t>(this);
    m_input_port->init(init_config->input_port_config);

    //! Step 4: Initialize publishers
    RDX_INFO_DEV(this, __func__, false, "{}", "Initializing publishers");
    m_pub_relayed_document.init(this, init_config->publish_topic, StampedImagePub::DefaultQoS);
    m_pub_relayed_image.init(this, init_config->publish_topic_image, StampedImagePub::DefaultQoS);
    m_pub_debug_document_accepted.init(this, init_config->debug_topic_document_accepted, StampedImagePub::DefaultUnreliableQoS);
    m_pub_debug_document_rejected.init(this, init_config->debug_topic_document_rejected, StampedImagePub::DefaultUnreliableQoS);


    // 创建中间结果保存路径
    if (init_config->enable_save_middle_result && !init_config->save_middle_result_dir_path.empty()) {
        std::string save_middle_result_dir_path = init_config->save_middle_result_dir_path;
        if (save_middle_result_dir_path.empty()) {
            RDX_RAISE_ERROR("[{}] save_middle_result_dir_path is empty", __func__);
        }
        std::filesystem::create_directories(save_middle_result_dir_path);
    }

    RDX_INFO_DEV(this, __func__, false, "{}", "Init config update completed successfully");
    return 0;
}

int PSGDocumentSink::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "updating runtime config");
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(config);
    if (!runtime_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast runtime config to FrameRelayNodeRuntimeConfig", __func__);
    }

    // {
    //     RDX_INFO_DEV(this, __func__, false, "{}", "Converting runtime config to JSON");
    //     auto runtime_config_json = JS::serializeStruct(*runtime_config);
    //     RDX_INFO_DEV(this, __func__, false, "RuntimeConfig JSON: {}", runtime_config_json);
    // }

    return 0;
}

int PSGDocumentSink::_start()
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting frame relay node");

    m_input_port->start();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port started");

    return 0;
}

int PSGDocumentSink::_stop()
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Stopping frame relay node");

    m_input_port->stop();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port stopped");

    return 0;
}

void PSGDocumentSink::_step()
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(get_runtime_config());
    // get data from input port
    // RDX_INFO_DEV(this, __func__, false, "{}", "getting data from input port");
    std::shared_ptr<SourceData_t> psg_document_data;
    if (runtime_config->enable_blocking_mode) {
        // wait until there is data available
        psg_document_data = m_input_port->pop_source_data();
    } else {
        // try to get data without waiting
        psg_document_data = m_input_port->try_pop_source_data();
    }

    // no frame data found
    if (!psg_document_data) {
        // RDX_INFO_DEV(this, __func__, false, "{}", "no frame data found");
        return;
    }

    // get goal handle
    RDX_INFO_DEV(this, __func__, false, "{}", "frame received, getting goal handle");
    auto goal_handle = psg_document_data->get_goal_handle_future().get();
    auto goal_uuid = to_boost_uuid(psg_document_data->get_goal_uuid());
    if (!goal_handle) {
        RDX_INFO_DEV(this, __func__, false, "[goal_uuid={}] {}",
                     boost::uuids::to_string(goal_uuid), "no goal handle found");
        return;
    }

    // get document and publish
    const auto msg_uuid = ActionDataTrait_t::get_uuid(*goal_handle->get_goal());
    const auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] {}",
                 msg_uuid_str, "psg document received and goal resolved");
    auto &raw_document = goal_handle->get_goal()->document;

    auto control_signal_code = ActionDataTrait_t::get_control_signal_code(*goal_handle->get_goal());

    m_pub_relayed_document.publish(raw_document);

    // 中间结果相关
    RDX_INFO_DEV(this, __func__, false, "{}", "开始处理中间结果");
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(m_init_config);
    if (!init_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast init config to FrameRelayNodeInitConfig", __func__);
    }
    if (init_config->enable_save_middle_result) {
        RDX_INFO_DEV(this, __func__, false, "{}", "中间结果保存已启用");
        if (init_config->save_middle_result_dir_path.empty()) {
            RDX_RAISE_ERROR("[{}] save_middle_result_dir_path is empty", __func__);
        } else {
            RDX_INFO_DEV(this, __func__, false, "{}", "开始保存每帧人员信息");
            _save_person_every_frame(init_config, raw_document);

            RDX_INFO_DEV(this, __func__, false, "{}", "开始保存每个人的轨迹信息");
            _save_trajectory_every_person(init_config, raw_document);

            RDX_INFO_DEV(this, __func__, false, "{}", "开始保存事件数据");
            _save_event_data(init_config, raw_document);

            // FIXME: 保存客流事件统计(flush或terminate时？还是应该在stop时？)
            if (control_signal_code == ControlSignalCode::Flush || control_signal_code == ControlSignalCode::Terminate) {
                RDX_INFO_DEV(this, __func__, false, "{}", "收到flush或terminate信号,开始保存事件统计");
                _save_event_count(init_config);
            }
        }
    } else {
        RDX_INFO_DEV(this, __func__, false, "{}", "中间结果保存未启用,跳过保存步骤");
    }
    // // Convert it into a YAML representation
    // RosMessage_Cpp ros_msg;
    // // Note: type support info could be retrieved through other means, see dynmsg::cpp::*
    // InterfaceTypeName interface {
    //     "psg_private_msgs", "PsgDocument"
    // };
    // ros_msg.type_info = dynmsg::cpp::get_type_info(interface);
    // ros_msg.data = reinterpret_cast<uint8_t *>(const_cast<psg_private_msgs::msg::PsgDocument *>(&raw_document));
    // YAML::Node yaml_msg = dynmsg::cpp::message_to_yaml(ros_msg);
    // RDX_INFO_DEV(this, __func__, false, "{}", "save yaml to file");
    // std::ofstream ofs("/3d/chengxiao/code/psf_ros2_ws/test.yaml", std::ios::out);
    // ofs << yaml_msg;
    // ofs.close();

    // publish image
    image_utils::FrameMediator fm(&raw_document.frame_bundle.primary_frame);
    sensor_msgs::msg::Image raw_image;
    fm.to_image_msg(raw_image);

    // publish debug topic?
    if (runtime_config->enable_debug_topics) {
        m_pub_debug_document_accepted.publish(raw_image, "accepted");
        RDX_INFO_DEV(this, __func__, false, "{}", "published debug document accepted");
    }

    // at the end, terminate the goal
    auto result_msg = std::make_shared<InputPort_t::ActionResult_t>();
    goal_handle->abort(result_msg);

    // 关闭整个ros2节点
    if (control_signal_code == ControlSignalCode::Flush || control_signal_code == ControlSignalCode::Terminate) {
        // 休眠10s
        std::this_thread::sleep_for(std::chrono::seconds(10));
        rclcpp::shutdown();
    }
}

void PSGDocumentSink::_save_event_count(std::shared_ptr<InitConfig_t> init_config)
{
    std::ofstream fw;
    std::string save_middle_result_dir_path = init_config->save_middle_result_dir_path;
    std::string result_save_path = save_middle_result_dir_path + "/count.txt"; // save final count result
    fw.open(result_save_path, std::ios_base::out);
    for (auto &iter : m_event_count) {
        fw << iter.first << std::endl;
        std::cout << iter.first << std::endl;
        for (auto &event : iter.second) {
            fw << " " << event.first << " " << event.second << std::endl;
            std::cout << " " << event.first << " " << event.second << std::endl;
        }
    }
    fw.close();
}

void PSGDocumentSink::_save_event_data(std::shared_ptr<InitConfig_t> init_config,
                                       const psg_private_msgs::msg::PsgDocument &raw_document)
{
    // 保存客流事件数据和统计
    std::ofstream fw_id_event;
    std::string save_middle_result_dir_path = init_config->save_middle_result_dir_path;
    std::string result_id_save_path = save_middle_result_dir_path + "/id_event.txt"; // save person id events
    fw_id_event.open(result_id_save_path, std::ios_base::app);

    // 统计客流事件
    for (auto &event_msg : raw_document.trajectory_events) {
        std::string event_zone_name = event_msg.event_zone_name;
        if (m_event_count.find(event_zone_name) == m_event_count.end()) {
            m_event_count[event_zone_name] = {{"In", 0}, {"Out", 0}};
        }
        std::string event_string = m_EventTyp2String[event_msg.event_type];
        if (event_string == "DoorIn" || event_string == "DoorSpeedIn" || event_string == "PassingIn")
            m_event_count[event_zone_name]["In"] += 1;
        if (event_string == "DoorOut" || event_string == "DoorSpeedOut" || event_string == "PassingOut")
            m_event_count[event_zone_name]["Out"] += 1;
        std::stringstream ss;
        ss << "id: " << event_msg.track_id << " event_zone_name: " << event_msg.event_zone_name << " event_type: " << event_msg.event_type << " " << event_string
           << " start_time: " << event_msg.start_time << " end_time: " << event_msg.end_time
           << " event_info: " << event_msg.matched_trajectory << " event_pattern: " << event_msg.event_pattern
           << " speed: " << event_msg.speed_2.x << " " << event_msg.speed_2.y << std::endl;
        std::cout << ss.str() << std::endl;
        fw_id_event << ss.str();
    }
    fw_id_event.close();
}

void PSGDocumentSink::_save_person_every_frame(std::shared_ptr<InitConfig_t> init_config,
                                               const psg_private_msgs::msg::PsgDocument &raw_document)
{
    std::string save_middle_result_dir_path = init_config->save_middle_result_dir_path;
    std::string persons_data_save_dir = save_middle_result_dir_path + "/persons";
    std::filesystem::create_directories(persons_data_save_dir);
    if (raw_document.persons.size() > 0) {
        int temp_frame_number = raw_document.frame_bundle.primary_frame.metadata.frame_num;
        std::string save_json_path = persons_data_save_dir + "/frame_" + std::to_string(temp_frame_number) + ".json";
        std::ofstream fw;
        fw.open(save_json_path);
        std::cout << "open: " << fw.is_open() << std::endl;
        nlohmann::json output_content;
        for (auto &person : raw_document.persons) {
            nlohmann::json person_json;
            person_json["id"] = person.track_id;

            // std::cout<<" save json path: "<<save_json_path<<" id "<<person.id<<std::endl;
            if (person.true_body.category == 0) {
                auto &body_bbox = person.true_body.bbox;
                person_json["body"] = {body_bbox.x, body_bbox.y, body_bbox.width, body_bbox.height};
            }
            if (person.true_head.category == 1) {
                auto &head_bbox = person.true_head.bbox;
                person_json["head"] = {head_bbox.x, head_bbox.y, head_bbox.width, head_bbox.height};
                person_json["height"] = person.body_height;
            }
            if (person.true_face.category == 2) {
                auto &face_bbox = person.true_face.bbox;
                person_json["face"] = {face_bbox.x, face_bbox.y, face_bbox.width, face_bbox.height};
            }
            if (person.true_body.keypoints.keypoints_2.size() > 0) {
                auto &pose = person.true_body.keypoints;
                for (int i = 0; i < pose.semantic_type.size(); i++) {
                    person_json["body_keypoints"].push_back(pose.keypoints_2[i].x);
                    person_json["body_keypoints"].push_back(pose.keypoints_2[i].y);
                    person_json["body_keypoints_conf"].push_back(pose.confidence[i]);
                }
            }
            // TODO:暂时不存人头脚点位置，如果需要，那需要修改在counter里面计算好保存到document的person里面
            // if (person.head_position_3.x != 0 && person.head_position_3.y != 0 && person.head_position_3.z != 0) {
            //     PassengerFlow::fVECTOR_3 head_position_vec{person.head_position[0], person.head_position[1],
            //                                                person.head_position[2]};
            //     auto head_position_uv = m_camera->project_points(head_position_vec);
            //     int u = head_position_uv[0], v = head_position_uv[1];
            //     person_json["head_position"] = {u, v};
            // }
            // if (person.has_foot_position) {
            //     PassengerFlow::fVECTOR_3 foot_position_vec{person.foot_position[0], person.foot_position[1],
            //                                                person.foot_position[2]};
            //     auto foot_position_uv = m_camera->project_points(foot_position_vec);
            //     int u = foot_position_uv[0], v = foot_position_uv[1];
            //     person_json["foot_position"] = {u, v};
            // }
            output_content.push_back(person_json);
        }
        // std::cout<<"output content: "<<output_content.size()<<std::endl;
        fw << std::setw(4) << output_content;
        fw.close();
    }
}

void PSGDocumentSink::_save_trajectory_every_person(std::shared_ptr<InitConfig_t> init_config,
                                                    const psg_private_msgs::msg::PsgDocument &raw_document)
{
    std::string save_middle_result_dir_path = init_config->save_middle_result_dir_path;
    std::string m_trajs_save_dir = save_middle_result_dir_path + "/trajs";
    std::filesystem::create_directories(m_trajs_save_dir);
    for (auto &person_traj : raw_document.trajectories) {
        auto person_id = person_traj.track_id;
        std::string save_json_path = m_trajs_save_dir + "/person_" + std::to_string(person_id) + ".json";
        std::ofstream fw;
        fw.open(save_json_path);
        nlohmann::json output_content;
        for (auto &person : person_traj.persons) {
            nlohmann::json person_json;
            person_json["id"] = person_id;
            person_json["frameNum"] = person.frame_metadata.frame_num;
            person_json["height"] = person.body_height;
            if (person.true_body.category == 0) {
                auto &body_bbox = person.true_body.bbox;
                person_json["body"] = {body_bbox.x, body_bbox.y, body_bbox.width, body_bbox.height};
                // if (person.true_body.is_detected_by_camera)
                person_json["body_detected_by_camera"] = 1;
                // else
                //     person_json["body_detected_by_camera"] = 0;
            }

            if (person.true_head.category == 1) {
                auto &head_bbox = person.true_head.bbox;
                person_json["head"] = {head_bbox.x, head_bbox.y, head_bbox.width, head_bbox.height};
                // if (person.person_head.is_detected_by_camera)
                person_json["head_detected_by_camera"] = 1;
                // else
                //     person_json["head_detected_by_camera"] = 0;
            }

            // TODO: 暂时不存人头脚点位置，如果需要，那需要修改在counter里面计算好保存到document的person里面
            // if (person.has_foot_position) {
            //     PassengerFlow::fVECTOR_3 foot_position_vec{person.foot_position[0], person.foot_position[1],
            //                                                person.foot_position[2]};
            //     auto foot_position_uv = m_camera->project_points(foot_position_vec);
            //     int u = foot_position_uv[0], v = foot_position_uv[1];
            //     person_json["foot_position"] = {u, v};
            // }

            // if (person.has_head_position) {
            //     PassengerFlow::fVECTOR_3 head_position_vec{person.head_position[0], person.head_position[1],
            //                                                person.head_position[2]};
            //     auto head_position_uv = m_camera->project_points(head_position_vec);
            //     int u = head_position_uv[0], v = head_position_uv[1];
            //     person_json["head_position"] = {u, v};
            // }

            if (person.true_body.keypoints.semantic_type.size() > 0) {
                auto &pose = person.true_body.keypoints;
                for (int i = 0; i < pose.semantic_type.size(); i++) {
                    person_json["body_keypoints"].push_back(pose.keypoints_2[i].x);
                    person_json["body_keypoints"].push_back(pose.keypoints_2[i].y);
                    person_json["body_keypoints_conf"].push_back(pose.confidence[i]);
                }
            }
            output_content.push_back(person_json);
        }

        fw << std::setw(4) << output_content;
    }
}

} // namespace redoxi_works
