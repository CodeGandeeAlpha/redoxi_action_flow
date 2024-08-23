#include <psg_common/msg_converter.hpp>
#include <psg_common/psg_common.hpp>

#include <rcpputils/asserts.hpp>
#include <tracker/_tracker.hpp>
#include <tracker/tracker.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
Tracker::Tracker()
    : Node("tracker_node")
{
    m_impl = std::make_shared<TrackerImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_track_targets_waiting_map = &m_track_targets_task_waiting;
    m_impl->sync_track_targets_doing_map = &m_track_targets_task_doing;

    m_impl->sync_detections_map = &m_detections_buffer;
    m_impl->sync_track_targets_map = &m_track_targets_buffer;

    RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int Tracker::init(const std::shared_ptr<InitConfig> &config,
                  const std::shared_ptr<RuntimeConfig> &runtime_config)
{
    if (m_status_code != NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::STOPPED) {
        RCLCPP_ERROR(m_impl->logger, "init FAILED! status code is not BEFORE_INIT or STOPPED");
        return ReturnCode::ERROR;
    }
    ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT,
               "init FAILED! status code is not BEFORE_INIT");

    m_init_config = config;
    m_runtime_config = runtime_config;

    // connect to v6d
    m_impl->v6d_client = create_v6d_client();

    // create process detections server
    m_act_process_detections = rclcpp_action::create_server<ACT_AcceptDetections>(
        this, m_init_config->process_detections_action,
        std::bind(&Tracker::_accept_detections_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Tracker::_accept_detections_cancel_callback, this, std::placeholders::_1),
        std::bind(&Tracker::_accept_detections_accepted_callback, this, std::placeholders::_1));

    // setup downstreams
    _connect_to_downstreams();

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<Tracker::InitConfig> &Tracker::get_init_config() const
{
    return m_init_config;
}

int Tracker::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<Tracker::RuntimeConfig> &Tracker::get_runtime_config() const
{
    return m_runtime_config;
}

int Tracker::open()
{
    // check status
    // you can open only if the node is initialized or closed
    ROS_ASSERT(m_status_code == NodeStatusCode::INITIALIZED || m_status_code == NodeStatusCode::CLOSED,
               "cannot open because status code is not INITIALIZED or CLOSED");
    ROS_ASSERT(m_impl->v6d_client != nullptr, "v6d_client is nullptr");
    ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");
    ROS_ASSERT(m_init_config->tracker_type == TrackerTypes::DEEPSORT || m_init_config->tracker_type == TrackerTypes::BOTSORT,
               "unsupported tracker type");

    if (m_init_config->tracker_type == TrackerTypes::DEEPSORT) {
        m_impl->tracker = std::make_shared<RedoxiTrack::DeepSortTracker>();
        m_impl->tracker_param = std::make_shared<RedoxiTrack::DeepSortTrackerParam>();
        m_impl->tracker->init(*m_impl->tracker_param);
        // not implemented event handler

    } else if (m_init_config->tracker_type == TrackerTypes::BOTSORT) {
        m_impl->tracker = std::make_shared<RedoxiTrack::BotsortTracker>();
        m_impl->tracker_param = std::make_shared<RedoxiTrack::BotsortTrackerParam>();
        m_impl->tracker->init(*m_impl->tracker_param);
        // not implemented event handler
    }

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::OPENED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);

    return ReturnCode::SUCCESS;
}


int Tracker::start()
{
    // the node must be opened
    ROS_ASSERT(m_status_code == NodeStatusCode::OPENED,
               "cannot start because status code is not OPENED");

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STARTED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);

    m_impl->step_running = true;
    m_impl->step_thread = std::make_shared<std::thread>(
        [this]() {
            while (rclcpp::ok() && m_impl->step_running) {
                _step();
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
            }
        });

    m_impl->process_thread = std::make_shared<std::thread>(
        [this]() {
            while (rclcpp::ok() && m_impl->step_running) {
                _process_step();
            }
        });

    return ReturnCode::SUCCESS;
}

int Tracker::stop()
{
    // only stoppable if the node is started
    ROS_ASSERT(m_status_code == NodeStatusCode::STARTED,
               "cannot stop because status code is not STARTED");

    // terminate step thread
    m_impl->step_running = false;
    if (m_impl->step_thread) {
        m_impl->step_thread->join();
        m_impl->step_thread = nullptr;
    }

    // terminate process thread
    if (m_impl->process_thread) {
        m_impl->process_thread->join();
        m_impl->process_thread = nullptr;
    }

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STOPPED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

int Tracker::close()
{
    // stop it if the node is running
    if (m_status_code == NodeStatusCode::STARTED)
        stop();

    // only valid if the node is opened or stopped
    ROS_ASSERT(m_status_code == NodeStatusCode::OPENED || m_status_code == NodeStatusCode::STOPPED,
               "[OpencvVideoReader] cannot close because status code is not OPENED or STOPPED");

    // closing, release v6d client
    m_impl->v6d_client->Disconnect();
    m_impl->v6d_client = nullptr;

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::CLOSED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);

    return ReturnCode::SUCCESS;
}


int Tracker::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse Tracker::_accept_detections_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDetections::Goal> goal)
{
    RCLCPP_INFO(m_impl->logger, "Received goal request with detections with frame_num %ld", goal->detections.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Tracker::_accept_detections_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void Tracker::_accept_detections_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDetections>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the detections
    const auto &detections = goal->detections;

    // add it to detections buffer
    {
        auto lock_ptr_detections_map = m_impl->sync_detections_map.synchronize();
        _add_detections_to_buffer(detections, *lock_ptr_detections_map);
    }

    RCLCPP_INFO(m_impl->logger, "_accept_detections_accepted_callback(): Accepted frame_number %ld and add it to buffer", detections.frame.frame_num);

    auto result = std::make_shared<ACT_AcceptDetections::Result>();
    result->return_msg = "Bodyposes accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}


void Tracker::_process_track_targets_create_tasks(const MSG_TrackTargets &track_targets, const MSG_Frame &frame,
                                                  Map_TrackTargets_Waiting *track_targets_waiting_map_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_process_track_targets_create_tasks(): create tasks for track targets %ld", frame.frame_num);

    // create tasks of this frame for all downstreams
    for (auto &x : m_downstreams) {
        auto task = std::make_shared<DSTask_TrackTargets>();
        task->downstream = x.second;
        task->track_targets = track_targets;
        task->frame = frame;
        (*track_targets_waiting_map_ptr)[std::make_tuple(task->downstream.get(), frame.frame_num)] = task;
    }
}


void Tracker::_step()
{

    _send_track_targets_to_downstreams();
}

void Tracker::_process_step()
{
    std::vector<std::pair<int, MSG_Detections>> detections_;
    {
        auto lock_ptr_detections_map = m_impl->sync_detections_map.synchronize();
        for (auto &it : **lock_ptr_detections_map) {
            detections_.push_back(std::make_pair(it.first, it.second));
        }
    }
    // sort detections by frame number
    std::sort(detections_.begin(), detections_.end(),
              [](std::pair<int, MSG_Detections> &a, std::pair<int, MSG_Detections> &b) {
                  return a.first < b.first;
              });

    // process detections
    for (auto &it : detections_) {
        auto &frame_num = it.first;
        auto &detections = it.second;

        auto &frame = detections.frame;

        // // get frame from shared memory
        // auto img = get_img_by_v6d_id(frame.cache.id_int, m_impl->v6d_client);

        // // track by detections
        // if (frame_num == 0) { // first frame
        //     std::vector<PassengerFlow::DetectionPtr> body_detections;
        //     convert_msg_to_detections(detections.detections, body_detections);
        //     m_impl->tracker->begin_track(img, body_detections, frame_num);

        //     auto all_open_targets = m_impl->tracker->get_all_open_targets();
        // }
    }
}

void Tracker::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

    m_downstreams.clear();

    for (auto it : m_init_config->pipeline_downstreams) {
        auto ds = std::make_shared<Downstream>();
        RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

        // 创建downstream
        {
            std::string name = it.second.accept_track_targets_action;
            auto client = rclcpp_action::create_client<ACT_AcceptTrackTargets>(this, name);

            ds->accept_track_targets = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&Tracker::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&Tracker::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&Tracker::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_downstreams[it.first] = ds;
    }
}

void Tracker::_send_track_targets_to_downstreams()
{
    std::vector<Map_TrackTargets_Waiting::key_type> tasks_to_remove;
    std::vector<std::pair<std::tuple<FlowRos2Pipeline::Tracker::Downstream *, int>,
                          std::shared_ptr<FlowRos2Pipeline::Tracker::DSTask_TrackTargets>>>
        track_targets_task_waiting_;

    {
        auto lock_ptr_track_targets_task_waiting = m_impl->sync_track_targets_waiting_map.synchronize();

        for (auto const &it : (**lock_ptr_track_targets_task_waiting)) {
            track_targets_task_waiting_.push_back(it);
        }
    }
    // sort track targets by frame number
    std::sort(track_targets_task_waiting_.begin(), track_targets_task_waiting_.end(),
              [](std::pair<std::tuple<FlowRos2Pipeline::Tracker::Downstream *, int>, std::shared_ptr<FlowRos2Pipeline::Tracker::DSTask_TrackTargets>> &a,
                 std::pair<std::tuple<FlowRos2Pipeline::Tracker::Downstream *, int>, std::shared_ptr<FlowRos2Pipeline::Tracker::DSTask_TrackTargets>> &b) {
                  return std::get<1>(a.first) < std::get<1>(b.first);
              });

    for (auto &it : track_targets_task_waiting_) {
        auto &task = it.second;
        ACT_AcceptTrackTargets::Goal goal;
        goal.track_targets = task->track_targets;
        goal.frame = task->frame;
        auto ds = task->downstream;
        auto handle = task->downstream->accept_track_targets->async_send_goal(goal, ds->accept_track_targets_options);

        // FIXME: add timeout condition
        auto task_response = handle.get();
        if (task_response != nullptr) {
            // accepted
            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
                // successfully sent, record this
                task->goal_handle = task_response;
                {
                    auto lock_ptr_track_targets_task_doing = m_impl->sync_track_targets_doing_map.synchronize();
                    (**lock_ptr_track_targets_task_doing)[task->goal_handle] = task;
                }
                tasks_to_remove.push_back(it.first);
            }

            // succeed
            else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                m_track_targets_task_done.push_back(task);
                tasks_to_remove.push_back(it.first);
            }
        }

        // FIXME: what if failed to send many times?
        // you need to terminate a frame, remove it from memory registry
    }

    // remove all sent tasks
    {
        auto lock_ptr_track_targets_task_waiting = m_impl->sync_track_targets_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_track_targets_task_waiting)->erase(it);
        }
    }

    {
        auto lock_ptr_track_targets_task_doing = m_impl->sync_track_targets_doing_map.synchronize();
        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_track_targets_task_doing)->empty()) {
            std::vector<GoalHandle_TrackTargets> tasks_to_remove;
            for (auto &it : (**lock_ptr_track_targets_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_track_targets_task_done.push_back(it.second);
                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_track_targets_task_doing)->erase(it);
            }
        }
    }

    // {
    //     auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
    //     // remove task done documents
    //     for (auto &it : m_psgdoc_task_done) {
    //         _remove_document_from_buffer(it->document.frame.frame_num, *lock_ptr_document_buffer);
    //     }
    // }

    // for all done tasks, remove them from memory
    m_track_targets_task_done.clear();
}


void Tracker::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_detections_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
}

void Tracker::_add_detections_to_buffer(const MSG_Detections &detections, Map_Detections *detections_map_ptr)
{
    (*detections_map_ptr)[detections.frame.frame_num] = detections;
}

void Tracker::_add_track_targets_to_buffer(const MSG_TrackTargets &track_targets, const MSG_Frame &frame, Map_TrackTargets *track_targets_map_ptr)
{
    (*track_targets_map_ptr)[frame.frame_num] = std::make_tuple(track_targets, frame);
}


void ROSTracker::init(const RedoxiTrack::TrackerParam &param)
{
    m_tracker = std::make_shared<RedoxiTrack::BotsortTracker>();
    m_tracker->init(param);
    m_track_event_handler = std::make_shared<TrackEventHandler>();
    m_tracker->add_event_handler(m_track_event_handler);
}

void ROSTracker::begin_track(const cv::Mat &img, const Tracker::MSG_Detections &detections, int frame_number)
{
    std::vector<RedoxiTrack::DetectionPtr> det_bodies;
    std::map<RedoxiTrack::DetectionPtr, std::pair<int, Tracker::MSG_Detection>> det_bodies2msg_detections;
    _msg_dets_to_body_dets(detections, det_bodies, det_bodies2msg_detections);

    m_track_event_handler->clear();
    m_tracker->begin_track(img, det_bodies, frame_number);
    for (auto &iter : m_track_event_handler->m_det2target_create) {
        auto track_det = iter.first;
        auto track_target = iter.second;
        auto detection_msg = det_bodies2msg_detections[track_det].second;

        // create track target msg
        Tracker::MSG_TrackTarget track_target_msg = _track_target_with_msg_det_to_msg(detection_msg, track_target);

        // create ROSAssociation data
        ROSTrackEvent::ROSAssociation evt_data;
        evt_data.from_index = det_bodies2msg_detections[track_det].first;
        evt_data.from = detection_msg;
        evt_data.to = track_target_msg;
        for (auto event_handler : m_ros_track_event_handler_set) {
            auto associated_status = event_handler->evt_ROS_track_create_pre(this, evt_data);
        }

        for (auto event_handler : m_ros_track_event_handler_set) {
            auto associated_status = event_handler->evt_ROS_track_create_after(this, evt_data);
        }
    }

    m_tracked_targets = m_tracker->get_all_open_targets();
}


void ROSTracker::track(const cv::Mat &img, const Tracker::MSG_Detections &detections, int frame_number)
{
    std::vector<RedoxiTrack::DetectionPtr> det_bodies;
    std::map<RedoxiTrack::DetectionPtr, std::pair<int, Tracker::MSG_Detection>> det_bodies2msg_detections;
    _msg_dets_to_body_dets(detections, det_bodies, det_bodies2msg_detections);

    m_track_event_handler->clear();
    m_tracker->track(img, det_bodies, frame_number);

    for (auto &iter : m_track_event_handler->m_det2target_create) {
        auto track_det = iter.first;
        auto track_target = iter.second;
        auto detection_msg = det_bodies2msg_detections[track_det].second;

        // create track target msg
        Tracker::MSG_TrackTarget track_target_msg = _track_target_with_msg_det_to_msg(detection_msg, track_target);

        // create ROSAssociation data
        ROSTrackEvent::ROSAssociation evt_data;
        evt_data.from_index = det_bodies2msg_detections[track_det].first;
        evt_data.from = detection_msg;
        evt_data.to = track_target_msg;
        for (auto event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_ROS_track_create_pre(this, evt_data);
        }

        for (auto event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_ROS_track_create_after(this, evt_data);
        }
    }

    for (auto &iter : m_track_event_handler->m_det2target_associate) {
        auto track_det = iter.first;
        auto track_target = iter.second;
        auto detection_msg = det_bodies2msg_detections[track_det].second;

        // create track target msg
        Tracker::MSG_TrackTarget track_target_msg = _track_target_with_msg_det_to_msg(detection_msg, track_target);

        // create ROSAssociation data
        ROSTrackEvent::ROSAssociation evt_data;
        evt_data.from_index = det_bodies2msg_detections[track_det].first;
        evt_data.from = detection_msg;
        evt_data.to = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_ROS_track_association_pre(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_ROS_track_association_after(this, evt_data);
        }
    }

    for (auto &target : m_track_event_handler->m_target_closed) {
        // create track target msg
        Tracker::MSG_TrackTarget track_target_msg = _track_target_to_msg(target);

        // create ROSClosed data
        ROSTrackEvent::ROSClosed evt_data;
        evt_data.target = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_trajectory_closed_before(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_trajectory_closed_after(this, evt_data);
        }
    }

    m_tracked_targets = m_tracker->get_all_open_targets();
}

void ROSTracker::track(const cv::Mat &img, int frame_number)
{
    m_track_event_handler->clear();
    m_tracker->track(img, frame_number);
    for (auto &target : m_track_event_handler->m_target_motion_predict) {
        // create track target msg
        Tracker::MSG_TrackTarget track_target_msg = _track_target_to_msg(target);

        // create ROSMotionPredict data
        ROSTrackEvent::ROSMotionPredict evt_data;
        evt_data.target = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_ROS_motion_predict_before(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_ROS_motion_predict_after(this, evt_data);
        }
    }

    for (auto &target : m_track_event_handler->m_target_closed) {
        // create track target msg
        Tracker::MSG_TrackTarget track_target_msg = _track_target_to_msg(target);

        // create ROSClosed data
        ROSTrackEvent::ROSClosed evt_data;
        evt_data.target = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_trajectory_closed_before(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_trajectory_closed_after(this, evt_data);
        }
    }

    m_tracked_targets = m_tracker->get_all_open_targets();
}

void ROSTracker::finish_track()
{
    m_track_event_handler->clear();
    m_tracker->finish_track();
    for (auto &target : m_track_event_handler->m_target_closed) {
        // create track target msg
        Tracker::MSG_TrackTarget track_target_msg = _track_target_to_msg(target);

        // create ROSClosed data
        ROSTrackEvent::ROSClosed evt_data;
        evt_data.target = track_target_msg;
        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_trajectory_closed_before(this, evt_data);
        }

        for (auto &event_handler : m_ros_track_event_handler_set) {
            auto status = event_handler->evt_trajectory_closed_after(this, evt_data);
        }
    }

    m_tracked_targets = m_tracker->get_all_open_targets();
}

void ROSTracker::add_event_handler(const ROSTrackEventHandlerPtr &person_event_handler)
{
    m_ros_track_event_handler_set.insert(person_event_handler);
}

void ROSTracker::remove_event_handler(const ROSTrackEventHandlerPtr &person_event_handler)
{
    m_ros_track_event_handler_set.erase(person_event_handler);
}

void ROSTracker::_msg_dets_to_body_dets(const Tracker::MSG_Detections &detections,
                                        std::vector<RedoxiTrack::DetectionPtr> &out_body_detections,
                                        std::map<RedoxiTrack::DetectionPtr, std::pair<int, Tracker::MSG_Detection>> &out_converted2before)
{
    for (int i = 0; i < detections.detections.size(); i++) {
        auto &det = detections.detections[i];
        if (det.category != 0)
            continue;
        // RedoxiTrack::DetectionPtr body_det = std::make_shared<RedoxiTrack::Detection>();
        PassengerFlow::DetectionPtr body_det = std::make_shared<PassengerFlow::Detection>();
        convert_msg_to_detection(det, body_det);
        out_body_detections.push_back(body_det);
        out_converted2before[body_det] = std::make_pair(i, det);
    }
}

Tracker::MSG_TrackTarget ROSTracker::_track_target_with_msg_det_to_msg(const Tracker::MSG_Detection &detection, RedoxiTrack::TrackTargetPtr track_target)
{
    Tracker::MSG_TrackTarget track_target_msg;
    track_target_msg.uuid = detection.uuid;
    track_target_msg.frame = detection.frame;
    track_target_msg.detection = detection;
    track_target_msg.track_id = track_target->get_path_id();
    track_target_msg.confidence = track_target->get_confidence();
    auto track_bbox = track_target->get_bbox();
    track_target_msg.track_bbox.x = track_bbox.x;
    track_target_msg.track_bbox.y = track_bbox.y;
    track_target_msg.track_bbox.width = track_bbox.width;
    track_target_msg.track_bbox.height = track_bbox.height;
    track_target_msg.track_status = track_target->get_path_state();

    return track_target_msg;
}

Tracker::MSG_TrackTarget ROSTracker::_track_target_to_msg(RedoxiTrack::TrackTargetPtr track_target)
{
    Tracker::MSG_TrackTarget track_target_msg;
    track_target_msg.track_id = track_target->get_path_id();
    track_target_msg.confidence = track_target->get_confidence();
    auto track_bbox = track_target->get_bbox();
    track_target_msg.track_bbox.x = track_bbox.x;
    track_target_msg.track_bbox.y = track_bbox.y;
    track_target_msg.track_bbox.width = track_bbox.width;
    track_target_msg.track_bbox.height = track_bbox.height;
    track_target_msg.track_status = track_target->get_path_state();

    return track_target_msg;
}

} // namespace FlowRos2Pipeline