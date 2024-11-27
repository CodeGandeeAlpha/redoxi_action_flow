#include <psg_tracker/PSGTracker.hpp>
#include <redoxi_common_cpp/redoxi_ros_util.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace redoxi_works
{
struct PSGTrackerImpl {

    //! ros time token
    std::shared_ptr<RosTimeToken> m_ros_time_token;

    //! tracker
    ROSTrackerPtr tracker;
    MyROSTrackEventHandlerPtr ros_track_event_handler;
};


PSGTracker::PSGTracker(const std::string &node_name, const rclcpp::NodeOptions &options)
    : common_nodes::OpenCloseNode(node_name, options)
{
}

int PSGTracker::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
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

    // create impl
    m_impl = _create_impl();

    //! Step 3: Create and initialize input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating input port");
    m_input_port = std::make_shared<InputPort_t>(this);
    m_input_port->init(init_config->input_port_config);

    RDX_INFO_DEV(this, __func__, false, "{}", "Init config update completed successfully");
    return 0;
}

int PSGTracker::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
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

    // enable debug topics?
    set_debug_topics_enabled(runtime_config->enable_debug_topics);

    return 0;
}

std::shared_ptr<PSGTrackerImpl> PSGTracker::_create_impl()
{
    // do not use init config or runtime config here, because it may not be initialized yet
    auto impl = std::make_shared<PSGTrackerImpl>();
    impl->m_ros_time_token = std::make_shared<RosTimeToken>(this);
    return impl;
}


int PSGTracker::_open()
{
    m_impl->tracker = std::make_shared<ROSTracker>();
    m_impl->ros_track_event_handler = std::make_shared<MyROSTrackEventHandler>();
    m_impl->tracker->add_event_handler(m_impl->ros_track_event_handler);
    if (m_init_config->tracker_type == psg_tracker::TrackerTypes::DEEPSORT) {
        auto deepsort_tracker_ptr = std::make_shared<RedoxiTrack::DeepSortTracker>();
        auto param = RedoxiTrack::DeepSortTrackerParam();
        deepsort_tracker_ptr->init(param);
        auto track_event_handler = std::make_shared<TrackEventHandler>();
        deepsort_tracker_ptr->add_event_handler(track_event_handler);

        m_impl->tracker->init(deepsort_tracker_ptr, track_event_handler);

    } else if (m_init_config->tracker_type == psg_tracker::TrackerTypes::BOTSORT) {
        auto botsort_tracker_ptr = std::make_shared<RedoxiTrack::BotsortTracker>();
        auto param = RedoxiTrack::BotsortTrackerParam();
        botsort_tracker_ptr->init(param);
        auto track_event_handler = std::make_shared<TrackEventHandler>();
        botsort_tracker_ptr->add_event_handler(track_event_handler);

        m_impl->tracker->init(botsort_tracker_ptr, track_event_handler);
    }

    return 0;
}

int PSGTracker::_close()
{
    return 0;
}

int PSGTracker::_start()
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting PSG tracker");

    if (m_input_port) {
        m_input_port->start();
        RDX_INFO_DEV(this, __func__, false, "{}", "input port started");
    }

    // create shm client
    {
        auto shm_config = shared_memory::SharedMemoryFactory::get_shm_config_from_node(this);
        m_shm_client = shared_memory::SharedMemoryFactory::create_client_by_config(shm_config);
        if (!m_shm_client) {
            RDX_INFO_DEV(this, __func__, false, "Failed to create shm client, service name = {}, region key = {}",
                         shm_config.service_name, shm_config.region_key);
        } else {
            RDX_INFO_DEV(this, __func__, false, "Created shm client, service name = {}, region key = {}",
                         shm_config.service_name, shm_config.region_key);
        }
    }

    // step thread and state will be handled by base class
    return 0;
}

int PSGTracker::_stop()
{
    m_input_port->stop();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port stopped");

    // delete shm client, disconnect from shm service
    m_shm_client.reset();

    return 0;
}

void PSGTracker::_step()
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    if (!runtime_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast runtime config to FrameRelayNodeRuntimeConfig", __func__);
    }

    // get data from input port
    // RDX_INFO_DEV(this, __func__, false, "{}", "getting data from input port");
    std::shared_ptr<SourceData_t> source_data;
    if (runtime_config->enable_blocking_mode) {
        // wait until there is data available
        source_data = m_input_port->pop_source_data();
    } else {
        // try to get data without waiting
        source_data = m_input_port->try_pop_source_data();
    }

    // no frame data found
    if (!source_data) {
        // RDX_INFO_DEV(this, __func__, false, "{}", "no frame data found");
        return;
    }

    // get goal handle
    RDX_INFO_DEV(this, __func__, false, "{}", "frame received, getting goal handle");
    auto goal_handle = source_data->get_goal_handle_future().get();
    auto goal_uuid = to_boost_uuid(source_data->get_goal_uuid());
    if (!goal_handle) {
        RDX_INFO_DEV(this, __func__, false, "[goal_uuid={}] {}",
                     boost::uuids::to_string(goal_uuid), "no goal handle found");
        return;
    }

    // flush signal?
    auto control_signal_code = ActionDataTrait_t::get_control_signal_code(*goal_handle->get_goal());
    if (control_signal_code == ControlSignalCode::Flush) {
        RDX_INFO_DEV(this, __func__, false, "[goal_uuid={}] {}",
                     boost::uuids::to_string(goal_uuid), "flush signal received");
    }

    // get frame and publish
    const auto msg_uuid = ActionDataTrait_t::get_uuid(*goal_handle->get_goal());
    const auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] {}",
                 msg_uuid_str, "frame received and goal resolved");


    // track
    auto track_targets = _track(source_data, control_signal_code);


    // publish debug topic?
    if (get_debug_topics_enabled()) {
        auto control_signal_code = ActionDataTrait_t::get_control_signal_code(*goal_handle->get_goal());
        auto label_text = fmt::format("accepted, signal = {}", control_signal_code_to_string(control_signal_code));
        m_pub_frame_accepted.publish(goal_handle->get_goal()->frame.raw_image, label_text);
    }

    // at the end, terminate the goal
    auto result_msg = std::make_shared<InputPort_t::ActionResult_t>();
    result_msg->track_targets = track_targets;
    goal_handle->succeed(result_msg);
}

int PSGTracker::_parse_frame(cv::Mat *output,
                             const SourceData_t &source_data)
{
    if (output == nullptr) {
        return 0;
    }

    auto msg_uuid = ActionDataTrait_t::get_uuid(*source_data.get_goal());

    // parse from shm
    auto &shm_token = source_data.get_goal()->frame.shm_token;
    if (shm_token.object_size >= 0 && m_shm_client && m_shm_client->is_connected()) {
        shared_memory::ObjectIdentifier oid;
        if (shm_token.object_id != 0) {
            oid.id = shm_token.object_id;
        }
        if (!shm_token.object_key.empty()) {
            oid.key = shm_token.object_key;
        }
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Getting data from shm with object id {}",
                     boost::uuids::to_string(msg_uuid), oid.id.value_or(0));
        auto datablock = m_shm_client->get_data(oid);
        if (!datablock) {
            RDX_INFO_DEV(this, __func__, false,
                         "[msg_uuid={}] Failed to get data from shm", boost::uuids::to_string(msg_uuid));
            return -1;
        }

        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Data from shm parsed to cv::Mat",
                     boost::uuids::to_string(msg_uuid));
        cv::Mat tmp;
        datablock->get_as_cvmat(&tmp);

        // IMPORTANT: copy the data to the output, because the datablock will be released after the function returns
        tmp.copyTo(*output);

        // after reading, delete the data from shm
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Deleting data from shm",
                     boost::uuids::to_string(msg_uuid));
        auto delete_ok = m_shm_client->delete_object(oid) == 0;
        if (!delete_ok) {
            RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Failed to delete data from shm",
                         boost::uuids::to_string(msg_uuid));
        }
    } else {
        // read raw image directly from the goal
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Reading raw image directly from the goal",
                     boost::uuids::to_string(msg_uuid));

        auto &raw_image = source_data.get_goal()->frame.raw_image;
        if (!raw_image.data.empty()) {
            auto img_bridge = cv_bridge::toCvCopy(raw_image, raw_image.encoding);
            *output = img_bridge->image;
        }
    }

    return 0;
}

std::vector<redoxi_public_msgs::msg::TrackTarget> PSGTracker::_track(const std::shared_ptr<SourceData_t> &source_data,
                                                                     const ControlSignalCode &control_signal_code)
{
    // get frame
    auto goal = source_data->get_goal();
    auto frame = goal->frame;
    auto frame_num = frame.metadata.frame_num;
    cv::Mat frame_mat;
    _parse_frame(&frame_mat, *source_data);

    // get detections
    auto persons = goal->persons;
    std::vector<redoxi_public_msgs::msg::Detection> detections;
    for (const auto &person : persons) {
        detections.push_back(person.body);
        detections.back().x_group_uid = person.x_uid; // 方便后面把track_targets和persons关联起来
    }

    std::vector<redoxi_public_msgs::msg::TrackTarget> track_targets;
    if (control_signal_code == ControlSignalCode::Flush || control_signal_code == ControlSignalCode::Terminate || frame_num == INT_MAX) { // last frame
        m_impl->ros_track_event_handler->clear();
        m_impl->tracker->finish_track();
        // put it in std::map<int, std::tuple<MSG_TrackTargets, MSG_Frame>>
        for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_closed) {
            track_targets.push_back(track_target_msg);
        }
    }

    else {
        // track by detections
        if (frame_num == 0) { // first frame
            m_impl->ros_track_event_handler->clear();
            m_impl->tracker->begin_track(frame_mat, detections, frame_num + 1);
            // put it in std::map<int, std::tuple<MSG_TrackTargets, MSG_Frame>>
            for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_create) {
                track_targets.push_back(track_target_msg);
            }
        } else if (frame_num != INT_MAX) { // track
            m_impl->ros_track_event_handler->clear();
            m_impl->tracker->track(frame_mat, detections, frame_num + 1);
            // put it in std::map<int, std::tuple<MSG_TrackTargets, MSG_Frame>>
            for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_create) {
                track_targets.push_back(track_target_msg);
            }
            for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_associate) {
                track_targets.push_back(track_target_msg);
            }

            for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_closed) {
                track_targets.push_back(track_target_msg);
            }
        }
    }
    return track_targets;
}

} // namespace redoxi_works
