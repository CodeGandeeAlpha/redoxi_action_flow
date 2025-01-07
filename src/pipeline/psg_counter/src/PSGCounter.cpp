#include <psg_counter/PSGCounter.hpp>
#include <psg_common/msg_converter.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <json_struct/json_struct.h>
#include <PassengerFlow/utils/util_functions.h>

#define PRINT_THREAD_ID_IN_LOG (true)

namespace redoxi_works
{

struct PSGCounterImpl {
    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    //! scene
    PassengerFlow::ScenePtr m_scene;
    //! ground
    PassengerFlow::GroundPtr m_ground;
    //! camera
    PassengerFlow::CameraModelPtr m_camera;
    //! event zones
    std::map<std::string, PassengerFlow::EventZonePtr> m_event_zones;
    //! spatial analyzer
    PassengerFlow::SingleGroundSpatialAnalyzer3Ptr m_spatial_analyzer;
    //! trajectory analyzer
    PassengerFlow::TrajectoryAnalyzerPtr m_trajectory_analyzer;

    //! psg counter functions
    SceneParameter _parse_config_file(const std::string &config_file_path);

    void _set_scene(const SceneParameter &scene_parameter, PassengerFlow::ScenePtr &out_scene,
                    PassengerFlow::CameraModelPtr &out_camera, PassengerFlow::GroundPtr &out_ground,
                    std::map<std::string, PassengerFlow::EventZonePtr> &out_event_zones);

    void _set_camera_model(const SceneParameter &scene_parameter, PassengerFlow::CameraModelPtr &cam);

    void _set_ground(const SceneParameter &scene_parameter, PassengerFlow::GroundPtr &ground);

    PassengerFlow::POINT _get_point_on_ground_from_project_point(const PassengerFlow::CameraModelPtr &cam,
                                                                 const PassengerFlow::GroundPtr &ground,
                                                                 const PassengerFlow::POINT &point);

    PassengerFlow::EventZonePtr _gen_event_zone(const RegionInfoPtr &region_info, const PassengerFlow::CameraModelPtr &cam,
                                                const PassengerFlow::GroundPtr &ground);

    PassengerFlow::EventZonePtr _gen_door_in_out_zone(const DoorInOutRegionInfo *region_info, const PassengerFlow::CameraModelPtr &cam,
                                                      const PassengerFlow::GroundPtr &ground);

    PassengerFlow::RegionPtr _gen_door_in_out_region(const std::vector<PassengerFlow::POINT> &door_line_points,
                                                     const PassengerFlow::POINT door_in_pixel_point,
                                                     const double buffer_area_length, const double certainly_area_length,
                                                     const PassengerFlow::CameraModelPtr &cam,
                                                     const PassengerFlow::GroundPtr &ground);

    PassengerFlow::EventZonePtr _gen_disappear_zone(const DisappearRegionInfo *region_info, const PassengerFlow::CameraModelPtr &cam,
                                                    const PassengerFlow::GroundPtr &ground);

    PassengerFlow::EventZonePtr _gen_passing_zone(const PassingRegionInfo *region_info, const PassengerFlow::CameraModelPtr &cam,
                                                  const PassengerFlow::GroundPtr &ground);

    void _draw_region_points(cv::Mat &img, const cv::Scalar &region_color, const std::string &region_name,
                             const std::vector<PassengerFlow::POINT> &region_points,
                             const PassengerFlow::GroundPtr &ground, const PassengerFlow::CameraModelPtr &camera);
    // pull input, work on it and then send output
    using PullProcessSendHandler_t = redoxi_works::port_handlers::PullProcessSendHandler<PSGCounter::InputPort_t::MasterSpec_t,
                                                                                         PSGCounter::OutputPort_t::MasterSpec_t>;
    std::shared_ptr<PullProcessSendHandler_t> work_then_send_handler;
};

PSGCounter::~PSGCounter()
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

int PSGCounter::_start()
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

int PSGCounter::_stop()
{
    //! Stop input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Stopping psg counter node");
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

void PSGCounter::set_publish_to_debug_topic(bool enable)
{
    m_publish_to_debug_topic = enable;
    if (m_publish_to_debug_topic) {
        if (m_primary_output_port) {
            m_primary_output_port->set_publish_to_debug_topic(enable);
        }
    }
}

bool PSGCounter::get_publish_to_debug_topic() const
{
    return m_publish_to_debug_topic;
}

int PSGCounter::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
{
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(config);

    // parse the config into a string and print it
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "parse init config into a string");
    auto config_str = JS::serializeStruct(*config);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "init config: {}", config_str);

    // create impl
    m_impl = _create_impl();

    // setup analyzers
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始解析乘客流配置文件: {}", init_config->passengerflow_config_path);
    auto param = m_impl->_parse_config_file(init_config->passengerflow_config_path);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "完成解析乘客流配置文件");
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "开始设置场景");
    m_impl->_set_scene(param, m_impl->m_scene, m_impl->m_camera, m_impl->m_ground, m_impl->m_event_zones);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "完成设置场景");
    m_impl->m_spatial_analyzer->set_scene(m_impl->m_scene);
    m_impl->m_spatial_analyzer->set_ground(m_impl->m_ground);

    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始设置事件区域到轨迹分析器, 共{}个区域", m_impl->m_event_zones.size());
    for (auto &iter : m_impl->m_event_zones) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "设置事件区域: {}", iter.first);
        m_impl->m_trajectory_analyzer->set_event_zone(iter.first, iter.second);
    }
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "完成设置所有事件区域到轨迹分析器");

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
        auto debug_qos = DefaultParams::get_debug_publisher_qos();
        m_pub_task_enqueue.init(this, init_config->debug_pub_task_enqueue_name, debug_qos);
        m_pub_task_drop.init(this, init_config->debug_pub_task_drop_name, debug_qos);
    }

    return 0;
}

int PSGCounter::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
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

std::shared_ptr<PSGCounterImpl> PSGCounter::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGCounterImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);

    impl->m_spatial_analyzer = std::make_shared<PassengerFlow::SingleGroundSpatialAnalyzer3>();
    impl->m_trajectory_analyzer = std::make_shared<PassengerFlow::TrajectoryAnalyzer>();
    impl->m_camera = std::make_shared<PassengerFlow::CameraModel>();
    impl->m_ground = std::make_shared<PassengerFlow::Ground>();
    impl->m_scene = std::make_shared<PassengerFlow::Scene>();
    return impl;
}

PSGCounter::DeliveryRequest_t
    PSGCounter::_create_delivery_request(const OutputSourceData_t &source_data,
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

std::shared_ptr<PSGCounter::OutputPort_t>
    PSGCounter::_create_primary_output_port(const InitConfig_t &init_config)
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

int PSGCounter::_create_document_request_handler(const RuntimeConfig_t &runtime_config)
{
    using ProcessHandler_t = PSGCounterImpl::PullProcessSendHandler_t;
    using InputDataTrait_t = PSGCounter::InputPort_t::ActionDataTrait_t;
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

            auto &trajectories = document_msg.trajectories;

            // count people
            // spatial analysis
            std::vector<PassengerFlow::PersonTrajectory> v_trajs;
            for (auto &msg_traj : trajectories) {
                PassengerFlow::PersonTrajectory traj;
                FlowRos2Pipeline::convert_msg_to_traj(msg_traj, traj);
                for (auto &person : traj.m_person_list) {
                    person->set_ground(m_impl->m_ground);
                    person->set_camera(m_impl->m_camera);
                    if (person->head()) {
                        dynamic_cast<PassengerFlow::Detection *>(person->head().get())->set_camera(m_impl->m_camera);
                        // 不加这个会导致没法计算身高，如果convert_msg_to_traj转出来是有头的说明person_msg里是有true_head的
                        dynamic_cast<PassengerFlow::Detection *>(person->head().get())->set_detected_by_camera(true);
                    }
                }
                v_trajs.push_back(traj);
            }

            m_impl->m_spatial_analyzer->process_inplace(v_trajs);
            // RCLCPP_DEBUG(m_impl->logger, "_process_step(): spatial analysis success!");

            // trajectory analysis
            std::vector<PassengerFlow::TrajectoryEvent> v_traj_event;

            // bool has_traj = false;

            for (auto &traj : v_trajs) {
                auto events = m_impl->m_trajectory_analyzer->process(traj);

                // test log
                for (auto &person : traj.m_person_list) {
                    if (person->body()) {
                        PassengerFlow::POINT3 foot_position;
                        bool position_valid;
                        person->get_foot_position(&foot_position, &position_valid);
                        RDX_INFO_DEV(this, __func__, false,
                                     "person id: {}, person height: {}, foot position: ({}, {}, {})",
                                     person->get_person_id(),
                                     person->get_body_height().m_body_height,
                                     foot_position.x, foot_position.y, foot_position.z);
                        // 输出keypoints
                        for (auto &iter : person->get_keypoints()) {
                            RDX_INFO_DEV(this, __func__, false,
                                         "keypoint: [{}, {}], confidence: {}",
                                         iter.second.m_point.x,
                                         iter.second.m_point.y,
                                         iter.second.m_confidence);
                        }

                        // 输出head bbox
                        if (person->head()) {
                            auto head_bbox = person->head()->get_bbox();
                            RDX_INFO_DEV(this, __func__, false,
                                         "head bbox: [{}, {}, {}, {}], score: {}",
                                         head_bbox.x, head_bbox.y, head_bbox.width, head_bbox.height,
                                         person->head()->get_confidence());
                        }
                    }

                    // has_traj = true;
                }

                for (auto &iter : events) {
                    for (auto &event : iter.second) {
                        RDX_INFO_DEV(this, __func__, false, "event id: {}, id: {}, event type: {}, start time: {}, end time: {}, \
                                event info: {}, speed_x: {}, speed_y: {}",
                                     event.m_person_id, event.m_person_id, event.m_event_type, event.m_start_time,
                                     event.m_end_time, event.m_matched_trajectory.c_str(), event.m_speed.x, event.m_speed.y);
                        v_traj_event.push_back(event);
                    }
                }
            }

            std::vector<psg_private_msgs::msg::TrajectoryEvent> v_msg_traj_events;
            for (auto &event : v_traj_event) {
                psg_private_msgs::msg::TrajectoryEvent msg_traj_event;
                FlowRos2Pipeline::convert_event_to_msg(event, msg_traj_event);
                v_msg_traj_events.push_back(msg_traj_event);
            }
            document_msg.trajectory_events = v_msg_traj_events;

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

int PSGCounter::_process_document_request()
{
    auto ret = m_impl->work_then_send_handler->process_and_send();
    if (ret == PSGCounterImpl::PullProcessSendHandler_t::ProcessResult::Error) {
        RDX_INFO_DEV(this, __func__, false, "Failed to process image request, error code: {}", int(ret));
        return -1;
    } else if (ret == PSGCounterImpl::PullProcessSendHandler_t::ProcessResult::NoData) {
        //! No data available, skipping
        return 0;
    } else if (ret == PSGCounterImpl::PullProcessSendHandler_t::ProcessResult::Success) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Successfully processed image request");
        return 0;
    } else if (ret == PSGCounterImpl::PullProcessSendHandler_t::ProcessResult::NoResourceToken) {
        //! No resource token, skipping
        return 0;
    } else if (ret == PSGCounterImpl::PullProcessSendHandler_t::ProcessResult::FailedToSend) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Failed to send image request to downstream, do you have a downstream?");
        return 0;
    } else {
        RDX_RAISE_ERROR("[f={}] Unexpected process result: {}", __func__, int(ret));
        return -1;
    }
}

void PSGCounter::_step()
{
    if (m_input_port) {
        _process_document_request();
    }
}

SceneParameter PSGCounterImpl::_parse_config_file(const std::string &config_file_path)
{
    std::ifstream fr;
    fr.open(config_file_path);
    SceneParameter output;

    rcpputils::assert_true(fr.is_open(), "parse_config_file(): failed to open config file");
    if (!fr.is_open())
        return output;
    else {
        nlohmann::json config;
        fr >> config;

        output.m_camera_fx = config["camera_fx"];
        output.m_camera_fy = config["camera_fy"];
        output.m_camera_ux = config["camera_ux"];
        output.m_camera_uy = config["camera_uy"];
        output.m_video_path = config["video_path"].get<std::string>();
        output.m_image_width = config["image_width"];
        output.m_image_height = config["image_height"];

        std::vector<double> Twc_vector = config["camera_extrinsic_inv"];
        std::vector<double> Twg_vector = config["ground_to_world"];
        PassengerFlow::MATRIX_4d Twc;
        Twc << Twc_vector[0], Twc_vector[1], Twc_vector[2], Twc_vector[3],
            Twc_vector[4], Twc_vector[5], Twc_vector[6], Twc_vector[7],
            Twc_vector[8], Twc_vector[9], Twc_vector[10], Twc_vector[11],
            Twc_vector[12], Twc_vector[13], Twc_vector[14], Twc_vector[15];
        output.m_camera_extrinsic = Twc.inverse();
        output.m_ground_to_world << Twg_vector[0], Twg_vector[1], Twg_vector[2], Twg_vector[3],
            Twg_vector[4], Twg_vector[5], Twg_vector[6], Twg_vector[7],
            Twg_vector[8], Twg_vector[9], Twg_vector[10], Twg_vector[11],
            Twg_vector[12], Twg_vector[13], Twg_vector[14], Twg_vector[15];
        int region_size = config["region_infos"].size();
        for (int i = 0; i < region_size; ++i) {
            auto region_info = config["region_infos"][i];
            if (region_info["region_type"] == PassengerFlow::RegionTypes::DoorInOut) {
                auto region = std::make_shared<DoorInOutRegionInfo>();
                region->m_name = region_info["region_name"].get<std::string>();
                region->m_region_type = region_info["region_type"];
                std::vector<int> region_points = region_info["points"];
                PassengerFlow::POINT door_point1(region_points[0], region_points[1]);
                PassengerFlow::POINT door_point2(region_points[2], region_points[3]);
                PassengerFlow::POINT door_in_point(region_points[4], region_points[5]);
                region->m_door_line_pixel_points.push_back(door_point1);
                region->m_door_line_pixel_points.push_back(door_point2);
                region->m_door_in_pixel_point = door_in_point;
                region->m_certain_region_size = region_info["certain_region_size"];
                region->m_likely_region_size = region_info["likely_region_size"];
                output.m_regions.push_back(region);
            } else if (region_info["region_type"] == PassengerFlow::RegionTypes::DoorDisappear) {
                auto region = std::make_shared<DisappearRegionInfo>();
                region->m_name = region_info["region_name"].get<std::string>();
                region->m_region_type = region_info["region_type"];
                std::vector<int> region_points = region_info["points"];
                PassengerFlow::POINT door_point1(region_points[0], region_points[1]);
                PassengerFlow::POINT door_point2(region_points[2], region_points[3]);
                PassengerFlow::POINT door_in_point(region_points[4], region_points[5]);
                region->m_door_line_pixel_points.push_back(door_point1);
                region->m_door_line_pixel_points.push_back(door_point2);
                region->m_door_in_pixel_point = door_in_point;
                region->m_region_size = region_info["region_size"];
                output.m_regions.push_back(region);
            }
        }
        return output;
    }
}

void PSGCounterImpl::_set_scene(const SceneParameter &scene_parameter, PassengerFlow::ScenePtr &out_scene,
                                PassengerFlow::CameraModelPtr &out_camera, PassengerFlow::GroundPtr &out_ground,
                                std::map<std::string, PassengerFlow::EventZonePtr> &out_event_zones)
{
    _set_camera_model(scene_parameter, out_camera);
    _set_ground(scene_parameter, out_ground);
    out_scene->add_camera(out_camera);
    out_scene->add_ground(out_ground);
    for (auto &region_info : scene_parameter.m_regions) {
        auto event_zone = _gen_event_zone(region_info, out_camera, out_ground);
        auto event_zone_name = region_info->m_name;
        out_event_zones[event_zone_name] = event_zone;
    }
}

void PSGCounterImpl::_set_camera_model(const SceneParameter &scene_parameter, PassengerFlow::CameraModelPtr &cam)
{
    PassengerFlow::fMATRIX_3 k;
    k << scene_parameter.m_camera_fx, 0, scene_parameter.m_camera_ux,
        0, scene_parameter.m_camera_fy, scene_parameter.m_camera_uy,
        0, 0, 1; // cam params
    cam->set_projection_matrix(k.transpose());
    cam->set_extrinsic_matrix(scene_parameter.m_camera_extrinsic.transpose());
    cam->set_image_size(scene_parameter.m_image_width, scene_parameter.m_image_height);
}

void PSGCounterImpl::_set_ground(const SceneParameter &scene_parameter, PassengerFlow::GroundPtr &ground)
{
    auto Tgw = scene_parameter.m_ground_to_world.inverse();
    ground->set_global_coordinate_frame(scene_parameter.m_ground_to_world);
    ground->set_ground_coordinate_frame(Tgw);
}

PassengerFlow::POINT PSGCounterImpl::_get_point_on_ground_from_project_point(const PassengerFlow::CameraModelPtr &cam,
                                                                             const PassengerFlow::GroundPtr &ground,
                                                                             const PassengerFlow::POINT &point)
{
    PassengerFlow::fVECTOR_3 point_ray, point_ray_p0;
    PassengerFlow::fVECTOR_2 point_uv_vector(point.x, point.y);
    cam->ray_from_projected_points(point_uv_vector, &point_ray_p0, &point_ray);

    auto ground_p0 = ground->get_p0();
    PassengerFlow::fVECTOR_3 ground_p0_vector{ground_p0.x, ground_p0.y, ground_p0.z};
    PassengerFlow::fVECTOR_3 ground_normal = ground->get_normal();

    auto door_point_in_world = PassengerFlow::get_line_surface_intersection(point_ray_p0, point_ray,
                                                                            ground_p0_vector, ground_normal);
    float temp_x = door_point_in_world(0), temp_y = door_point_in_world(1), temp_z = door_point_in_world(2);
    auto door_point_on_ground = ground->world_to_ground({temp_x, temp_y, temp_z});
    return door_point_on_ground;
}

PassengerFlow::EventZonePtr PSGCounterImpl::_gen_event_zone(const RegionInfoPtr &region_info, const PassengerFlow::CameraModelPtr &cam,
                                                            const PassengerFlow::GroundPtr &ground)
{
    if (region_info->m_region_type == PassengerFlow::RegionTypes::DoorInOut) {
        auto door_in_region_info = RedoxiTrack::dyncast_with_check<DoorInOutRegionInfo>(region_info.get());
        auto event_zone = _gen_door_in_out_zone(door_in_region_info, cam, ground);
        return event_zone;
    } else if (region_info->m_region_type == PassengerFlow::RegionTypes::DoorDisappear) {
        auto disappear_region_info = RedoxiTrack::dyncast_with_check<DisappearRegionInfo>(region_info.get());
        auto event_zone = _gen_disappear_zone(disappear_region_info, cam, ground);
        return event_zone;
    } else if (region_info->m_region_type == PassengerFlow::RegionTypes::Passing) {
        auto passing_region_info = RedoxiTrack::dyncast_with_check<PassingRegionInfo>(region_info.get());
        auto event_zone = _gen_passing_zone(passing_region_info, cam, ground);
        return event_zone;
    } else {
        RedoxiTrack::assert_throw(false, "please set region type");
        return PassengerFlow::EventZonePtr();
    }
}

PassengerFlow::EventZonePtr PSGCounterImpl::_gen_door_in_out_zone(const DoorInOutRegionInfo *region_info, const PassengerFlow::CameraModelPtr &cam,
                                                                  const PassengerFlow::GroundPtr &ground)
{
    auto door_in_out_region = _gen_door_in_out_region(region_info->m_door_line_pixel_points, region_info->m_door_in_pixel_point,
                                                      region_info->m_likely_region_size, region_info->m_certain_region_size, cam, ground);
    auto door_in_out_region_name = region_info->m_name + " : door in out region";
    door_in_out_region->set_name(door_in_out_region_name);

    PassengerFlow::DoorInOutEventZonePtr door_in_out_event_zone = std::make_shared<PassengerFlow::DoorInOutEventZone>();
    door_in_out_event_zone->set_region(door_in_out_region);
    door_in_out_event_zone->set_name(region_info->m_name);
    return door_in_out_event_zone;
}

PassengerFlow::RegionPtr PSGCounterImpl::_gen_door_in_out_region(const std::vector<PassengerFlow::POINT> &door_line_points,
                                                                 const PassengerFlow::POINT door_in_pixel_point,
                                                                 const double buffer_area_length, const double certainly_area_length,
                                                                 const PassengerFlow::CameraModelPtr &cam,
                                                                 const PassengerFlow::GroundPtr &ground)
{
    auto door_point1_uv = door_line_points[0];
    auto door_point2_uv = door_line_points[1];
    auto door_point1_on_ground = _get_point_on_ground_from_project_point(cam, ground, door_point1_uv);
    auto door_point2_on_ground = _get_point_on_ground_from_project_point(cam, ground, door_point2_uv);
    auto door_in_point_on_ground = _get_point_on_ground_from_project_point(cam, ground, door_in_pixel_point);
    std::vector<PassengerFlow::POINT> door_points_on_ground{door_point1_on_ground, door_point2_on_ground};
    auto region = std::make_shared<PassengerFlow::DoorInOutRegion>(ground, door_points_on_ground,
                                                                   door_in_point_on_ground, buffer_area_length, certainly_area_length);
    return region;
}

PassengerFlow::EventZonePtr PSGCounterImpl::_gen_disappear_zone(const DisappearRegionInfo *region_info, const PassengerFlow::CameraModelPtr &cam,
                                                                const PassengerFlow::GroundPtr &ground)
{
    auto door_point1_uv = region_info->m_door_line_pixel_points[0];
    auto door_point2_uv = region_info->m_door_line_pixel_points[1];
    auto door_point1_on_ground = _get_point_on_ground_from_project_point(cam, ground, door_point1_uv);
    auto door_point2_on_ground = _get_point_on_ground_from_project_point(cam, ground, door_point2_uv);
    auto door_in_point_on_ground = _get_point_on_ground_from_project_point(cam, ground, region_info->m_door_in_pixel_point);
    std::vector<PassengerFlow::POINT> door_points_on_ground{door_point1_on_ground, door_point2_on_ground};
    double disappear_size = region_info->m_region_size;
    auto disappear_region = std::make_shared<PassengerFlow::DisappearInOutRegion>(ground, door_points_on_ground, door_in_point_on_ground, disappear_size);

    std::string disappear_region_name = region_info->m_name + " : disappear region";
    disappear_region->set_name(disappear_region_name);

    auto disappear_event_zone = std::make_shared<PassengerFlow::DisappearInOutEventZone>();
    disappear_event_zone->set_region(disappear_region);
    disappear_event_zone->set_name(region_info->m_name);
    return disappear_event_zone;
}

PassengerFlow::EventZonePtr PSGCounterImpl::_gen_passing_zone(const PassingRegionInfo *region_info, const PassengerFlow::CameraModelPtr &cam,
                                                              const PassengerFlow::GroundPtr &ground)
{

    std::vector<PassengerFlow::POINT> region_points_on_ground;
    for (auto &point_in_pixel : region_info->m_region_pixel_points) {
        auto point_on_ground = _get_point_on_ground_from_project_point(cam, ground, point_in_pixel);
        region_points_on_ground.push_back(point_on_ground);
    }

    auto passing_region = std::make_shared<PassengerFlow::PassingInOutRegion>(ground, region_points_on_ground);

    std::string passing_region_name = region_info->m_name + " : disappear region";
    passing_region->set_name(passing_region_name);

    auto passing_in_out_event_zone = std::make_shared<PassengerFlow::PassingInOutEventZone>();
    passing_in_out_event_zone->set_region(passing_region);
    passing_in_out_event_zone->set_name(region_info->m_name);
    return passing_in_out_event_zone;
}

void PSGCounterImpl::_draw_region_points(cv::Mat &img, const cv::Scalar &region_color, const std::string &region_name,
                                         const std::vector<PassengerFlow::POINT> &region_points,
                                         const PassengerFlow::GroundPtr &ground, const PassengerFlow::CameraModelPtr &camera)
{
    std::vector<cv::Point2i> region_points_on_img;
    int center_u = 0, center_v = 0;
    for (auto &point : region_points) {
        auto point_in_world = ground->ground_to_world(point);
        PassengerFlow::fVECTOR_3 point_in_world_vec(point_in_world.x, point_in_world.y, point_in_world.z);
        auto point_on_img = camera->project_points(point_in_world_vec);
        int u = point_on_img[0], v = point_on_img[1];
        center_u += u;
        center_v += v;
        region_points_on_img.push_back({u, v});
    }

    center_u /= region_points_on_img.size();
    center_v /= region_points_on_img.size();
    PassengerFlow::POINT pre_point = region_points_on_img[0], next_point;
    for (size_t i = 1; i < region_points_on_img.size(); ++i) {
        //! 获取下一个点
        next_point = region_points_on_img[i];
        RDX_INFO_DEV(nullptr, __func__, false, "绘制区域线段 - 从点[{},{}]到点[{},{}]",
                     pre_point.x, pre_point.y, next_point.x, next_point.y);
        //! 绘制线段
        cv::line(img, pre_point, next_point, region_color);
        //! 更新前一个点
        pre_point = next_point;
    }
    next_point = region_points_on_img[0];
    RDX_INFO_DEV(nullptr, __func__, false, "绘制区域线段 - 从点[{},{}]到点[{},{}]",
                 pre_point.x, pre_point.y, next_point.x, next_point.y);
    cv::line(img, pre_point, next_point, region_color);
    cv::putText(img, region_name, cv::Point2i(center_u, center_v), cv::FONT_HERSHEY_SIMPLEX, 1, region_color, 2);
}

//! 将document中的raw image转换为带有检测框和body keypoints的debug image
sensor_msgs::msg::Image PSGCounter::_create_debug_image(const psg_private_msgs::msg::PsgDocument &document)
{
    //! 转换raw image到cv::Mat
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始转换raw image到cv::Mat", 0);
    cv::Mat cv_image;
    image_utils::FrameMediator fm(&document.frame_bundle.primary_frame);
    fm.to_cv_image_copy(cv_image);

    // //! 在图像上画person相关的框和keypoints
    // RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始在图像上绘制person相关的框和keypoints, 共{}个person", document.persons.size());
    // for (const auto &person : document.persons) {
    //     //! 随机生成颜色
    //     cv::Scalar color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);

    //     //! 获取body bbox坐标
    //     if (person.body.category == 0) {
    //         int x = static_cast<int>(person.body.bbox.x);
    //         int y = static_cast<int>(person.body.bbox.y);
    //         int width = static_cast<int>(person.body.bbox.width);
    //         int height = static_cast<int>(person.body.bbox.height);

    //         //! 画body bbox
    //         cv::rectangle(cv_image,
    //                       cv::Point(x, y),
    //                       cv::Point(x + width, y + height),
    //                       color, 2);
    //     }

    //     //! 画body keypoints
    //     const auto &keypoints = person.body.keypoints;

    //     //! 在访问数组或指针前添加检查
    //     if (!keypoints.keypoints_2.empty() && !keypoints.confidence.empty()) {
    //         //! 画出17个关键点
    //         for (size_t i = 0; i < keypoints.keypoints_2.size(); i++) {
    //             if (keypoints.confidence[i] > 0.3) { // 只画置信度大于0.3的点
    //                 //! 记录关键点的位置和置信度
    //                 RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
    //                              "绘制关键点[{}] - 位置:[{}, {}], 置信度:{}",
    //                              i, keypoints.keypoints_2[i].x, keypoints.keypoints_2[i].y, keypoints.confidence[i]);
    //                 cv::circle(cv_image,
    //                            cv::Point(keypoints.keypoints_2[i].x, keypoints.keypoints_2[i].y),
    //                            3, color, -1);
    //             }
    //         }

    //         //! 画出骨架连接
    //         //! COCO数据集的17个关键点连接对
    //         const std::vector<std::pair<int, int>> skeleton = {
    //             {5, 7}, {7, 9}, {6, 8}, {8, 10}, // 手臂
    //             {11, 13},
    //             {13, 15},
    //             {12, 14},
    //             {14, 16}, // 腿
    //             {5, 6},
    //             {5, 11},
    //             {6, 12},  // 躯干
    //             {11, 12}, // 臀部
    //             {1, 2},
    //             {1, 3},
    //             {2, 4},
    //             {3, 5},
    //             {4, 6} // 头部和肩膀
    //         };

    //         for (const auto &bone : skeleton) {
    //             if (keypoints.confidence[bone.first] > 0.3 && keypoints.confidence[bone.second] > 0.3) {
    //                 //! 记录骨架连接的起点和终点
    //                 RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG,
    //                              "绘制骨架连接 - 从关键点[{},{}]到关键点[{},{}]",
    //                              keypoints.keypoints_2[bone.first].x, keypoints.keypoints_2[bone.first].y,
    //                              keypoints.keypoints_2[bone.second].x, keypoints.keypoints_2[bone.second].y);
    //                 cv::line(cv_image,
    //                          cv::Point(keypoints.keypoints_2[bone.first].x, keypoints.keypoints_2[bone.first].y),
    //                          cv::Point(keypoints.keypoints_2[bone.second].x, keypoints.keypoints_2[bone.second].y),
    //                          color, 2);
    //             }
    //         }
    //     }

    //     //! 画head bbox
    //     if (person.head.category == 1) {
    //         int x = static_cast<int>(person.head.bbox.x);
    //         int y = static_cast<int>(person.head.bbox.y);
    //         int width = static_cast<int>(person.head.bbox.width);
    //         int height = static_cast<int>(person.head.bbox.height);

    //         //! 画head bbox
    //         cv::rectangle(cv_image,
    //                       cv::Point(x, y),
    //                       cv::Point(x + width, y + height),
    //                       color, 2);
    //     }

    //     //! 画face bbox
    //     if (person.face.category == 2) {
    //         int x = static_cast<int>(person.face.bbox.x);
    //         int y = static_cast<int>(person.face.bbox.y);
    //         int width = static_cast<int>(person.face.bbox.width);
    //         int height = static_cast<int>(person.face.bbox.height);

    //         //! 画face bbox
    //         cv::rectangle(cv_image,
    //                       cv::Point(x, y),
    //                       cv::Point(x + width, y + height),
    //                       color, 2);
    //     }
    // }

    //! 画事件区域
    int rand_seed = 0;
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始绘制事件区域, 共{}个区域", m_impl->m_event_zones.size());
    for (auto &iter : m_impl->m_event_zones) {
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始绘制区域: {}", iter.first);

        //! 生成随机颜色
        srand(rand_seed);
        rand_seed += 10;
        int r = rand() % 255;
        int b = rand() % 255;
        cv::Scalar event_zone_color(b, 0, r);
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "区域颜色: B={}, R={}", b, r);

        //! 获取区域点集
        auto event_regions = iter.second->get_region_points();
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "区域包含{}个子区域", event_regions.size());

        //! 绘制每个子区域
        for (auto &region : event_regions) {
            RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "绘制子区域: {}", region.first);
            m_impl->_draw_region_points(cv_image, event_zone_color, region.first, region.second, m_impl->m_ground, m_impl->m_camera);
        }
        RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "完成区域{}的绘制", iter.first);
    }
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "{}", "完成所有事件区域的绘制");

    //! 转回sensor_msgs/Image
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "开始转换回sensor_msgs/Image", 0);
    sensor_msgs::msg::Image debug_image;
    image_utils::FrameMediator fm_cv_image(cv_image, "bgr8");
    fm_cv_image.to_image_msg(debug_image);
    RDX_INFO_DEV(this, __func__, PRINT_THREAD_ID_IN_LOG, "完成debug图像创建, 大小: {}x{}", debug_image.width, debug_image.height);
    return debug_image;
}

} // namespace redoxi_works
