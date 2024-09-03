#include <psg_common/msg_converter.hpp>
#include <psg_common/psg_common.hpp>

#include <rcpputils/asserts.hpp>
#include <tracker/_tracker.hpp>
#include <tracker/tracker.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
std::string bbox_msg_to_string(const psg_public_msgs::msg::Box2 &bbox)
{
    std::string str = "(";
    str += std::to_string(bbox.x) + ", ";
    str += std::to_string(bbox.y) + ", ";
    str += std::to_string(bbox.width) + ", ";
    str += std::to_string(bbox.height) + ")";

    return str;
}

std::string track_target_msg_to_string(const Tracker::MSG_TrackTarget &track_target)
{
    std::string str = "TrackTarget: {\n";
    str += "frame_num: " + std::to_string(track_target.frame.frame_num) + "\n";
    str += "uuid: " + uuid_to_string(track_target.uuid.uuid) + "\n";
    str += "track_id: " + std::to_string(track_target.track_id) + "\n";
    str += "track_status: " + std::to_string(track_target.track_status) + "\n";
    str += "confidence: " + std::to_string(track_target.confidence) + "\n";
    str += "track_bbox: " + bbox_msg_to_string(track_target.track_bbox) + "\n";
    str += "detection_bbox: " + bbox_msg_to_string(track_target.detection.bbox) + "\n";
    str += "}";

    return str;
}

std::string detection_msg_to_string(const Tracker::MSG_Detection &detection)
{
    std::string str = "Detection: {\n";
    str += "frame_num: " + std::to_string(detection.frame.frame_num) + "\n";
    str += "category: " + std::to_string(detection.category) + "\n";
    str += "uuid: " + uuid_to_string(detection.uuid.uuid) + "\n";
    str += "bbox: " + bbox_msg_to_string(detection.bbox) + "\n";
    str += "confidence: " + std::to_string(detection.confidence) + "\n";
    str += "}";

    return str;
}

std::string rect2f_to_string(const cv::Rect2f &rect)
{
    std::ostringstream oss;
    oss << "Rect2f: {"
        << "x: " << rect.x << ", "
        << "y: " << rect.y << ", "
        << "width: " << rect.width << ", "
        << "height: " << rect.height
        << "}";
    return oss.str();
}

std::string psg_detection_to_string(const RedoxiTrack::DetectionPtr &detection)
{
    std::string str = "Detection: {\n";
    str += "category: " + std::to_string(detection->get_type()) + "\n";
    str += "bbox: " + rect2f_to_string(detection->get_bbox()) + "\n";
    str += "confidence: " + std::to_string(detection->get_confidence()) + "\n";
    str += "}";

    return str;
}

std::string psg_track_target_to_string(const RedoxiTrack::TrackTargetPtr &track_target)
{
    std::string str = "TrackTarget: {\n";
    str += "track_id: " + std::to_string(track_target->get_path_id()) + "\n";
    str += "track_status: " + std::to_string(track_target->get_path_state()) + "\n";
    str += "track_bbox: " + rect2f_to_string(track_target->get_bbox()) + "\n";
    str += "detection_bbox: " + rect2f_to_string(track_target->get_underlying_detection()->get_bbox()) + "\n";
    str += "}";

    return str;
}

cv::Scalar _get_color(const int id)
{
    int idx = id * 3;
    cv::Scalar color((37 * idx) % 255, (17 * idx) % 255, (29 * idx) % 255);
    return color;
}

void draw_track_targets_msg_on_img(cv::Mat& img, const Tracker::MSG_TrackTargets &targets)
{
    RCLCPP_INFO(rclcpp::get_logger("tracker_node"), "draw_track_targets_msg_on_img()");
    for (const auto &target : targets) {
        cv::Rect2i bbox(int(target.track_bbox.x), int(target.track_bbox.y),
                      int(target.track_bbox.width), int(target.track_bbox.height));
        const auto color = _get_color(target.track_id);
        cv::rectangle(img, bbox, color, 3);

        cv::Point2i position(int(bbox.x), int(bbox.y));
        cv::putText(img, std::to_string(target.track_id), position, cv::FONT_HERSHEY_SIMPLEX, 1.2,
                    cv::Scalar(0, 0, 255), 2);
    }
}

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

    m_impl->out_video_writer.open("/mnt/chengxiao/tracker_test_out.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'V'),
                                  30,
                                  cv::Size(1920, 1080));

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

    m_impl->tracker = std::make_shared<ROSTracker>();
    m_impl->ros_track_event_handler = std::make_shared<MyROSTrackEventHandler>();
    m_impl->tracker->add_event_handler(m_impl->ros_track_event_handler);
    if (m_init_config->tracker_type == TrackerTypes::DEEPSORT) {
        auto deepsort_tracker_ptr = std::make_shared<RedoxiTrack::DeepSortTracker>();
        auto param = RedoxiTrack::DeepSortTrackerParam();
        deepsort_tracker_ptr->init(param);
        auto track_event_handler = std::make_shared<TrackEventHandler>();
        deepsort_tracker_ptr->add_event_handler(track_event_handler);

        m_impl->tracker->init(deepsort_tracker_ptr, track_event_handler);

    } else if (m_init_config->tracker_type == TrackerTypes::BOTSORT) {
        auto botsort_tracker_ptr = std::make_shared<RedoxiTrack::BotsortTracker>();
        auto param = RedoxiTrack::BotsortTrackerParam();
        botsort_tracker_ptr->init(param);
        auto track_event_handler = std::make_shared<TrackEventHandler>();
        botsort_tracker_ptr->add_event_handler(track_event_handler);

        m_impl->tracker->init(botsort_tracker_ptr, track_event_handler);
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
    auto detections = goal->detections;

    // add it to detections buffer
    {
        auto lock_ptr_detections_map = m_impl->sync_detections_map.synchronize();
        // if is empty frame, mark it as INT_MAX
        if (detections.frame.frame_num == -1) {
            detections.frame.frame_num = INT_MAX;
        }
        _add_detections_to_buffer(detections, *lock_ptr_detections_map);
    }

    RCLCPP_INFO(m_impl->logger, "_accept_detections_accepted_callback(): Accepted frame_number %ld and add it to buffer", detections.frame.frame_num);

    auto result = std::make_shared<ACT_AcceptDetections::Result>();
    result->return_msg = "Detections accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);

    RCLCPP_INFO(m_impl->logger, "_accept_detections_accepted_callback(): return client success in frame_number %ld", detections.frame.frame_num);
}


void Tracker::_process_track_targets_create_tasks(const MSG_TrackTargets &track_targets, const MSG_Frame &frame,
                                                  Map_TrackTargets_Waiting *track_targets_waiting_map_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_process_track_targets_create_tasks(): create tasks for track targets %ld", frame.frame_num);

    // // test log
    // for (const auto &track_target : track_targets) {
    //     RCLCPP_INFO(m_impl->logger, "_process_track_targets_create_tasks(): track_target %s",
    //                 track_target_msg_to_string(track_target).c_str());
    // }

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
    std::vector<std::tuple<MSG_TrackTargets, MSG_Frame>> track_targets_frames_;
    {
        auto lock_ptr_track_targets_map = m_impl->sync_track_targets_map.synchronize();
        for (auto &it : **lock_ptr_track_targets_map) {
            track_targets_frames_.push_back(it.second);
        }
    }
    for (auto &it : track_targets_frames_) {
        auto &track_targets = std::get<0>(it);
        auto &frame = std::get<1>(it);
        {
            auto lock_ptr_track_targets_task_waiting = m_impl->sync_track_targets_waiting_map.synchronize();
            _process_track_targets_create_tasks(track_targets, frame, *lock_ptr_track_targets_task_waiting);
        }
        {
            auto lock_ptr_track_targets_map = m_impl->sync_track_targets_map.synchronize();
            _remove_track_targets_from_buffer(frame.frame_num, *lock_ptr_track_targets_map);
        }
    }
    _send_track_targets_to_downstreams();
}

void Tracker::_process_step()
{
    std::vector<std::pair<int, MSG_Detections>> detections_;
    {
        auto lock_ptr_detections_map = m_impl->sync_detections_map.synchronize();

        for (auto &it : **lock_ptr_detections_map) {
            auto &frame_num = it.first;
            auto &detections = it.second;
            if (frame_num == m_waiting_frame_number) {
                m_waiting_frame_number++;
                detections_.push_back(it);
                RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d detections push_back to detections_", it.first);
                // // test log
                // for (const auto &detection : detections.detections) {
                //     RCLCPP_INFO(m_impl->logger, "_process_step(): detection %s",
                //                 detection_msg_to_string(detection).c_str());
                // }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
                break;
            }
        }
    }

    // process detections
    for (auto &it : detections_) {
        auto &frame_num = it.first;
        auto &detections = it.second;

        auto &frame = detections.frame;

        RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d", frame_num);

        // get frame from shared memory
        auto tensor = get_tensor_by_v6d_id(frame.cache.id_int, m_impl->v6d_client);
        auto img = from_v6d_tensor_to_cvmat(tensor);
        RCLCPP_INFO(m_impl->logger, "_process_step(): after from v6d tensor to cv mat");
        RCLCPP_INFO(m_impl->logger, "_process_step(): Image shape: rows = %d, cols = %d", img.rows, img.cols);

        MSG_TrackTargets cur_track_targets;
        // track by detections
        if (frame_num == 0) { // first frame
            m_impl->ros_track_event_handler->clear();
            RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d before begin_track", frame_num);
            m_impl->tracker->begin_track(img, detections, frame_num + 1);
            RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d begin_track", frame_num);
            // put it in std::map<int, std::tuple<MSG_TrackTargets, MSG_Frame>>
            for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_create) {
                cur_track_targets.push_back(track_target_msg);
            }

            // // test log
            RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d track_targets size %d", frame_num, cur_track_targets.size());
            // for (const auto &track_target : cur_track_targets) {
            //     RCLCPP_INFO(m_impl->logger, "_process_step(): track_target %s",
            //                 track_target_msg_to_string(track_target).c_str());
            // }

            // visiualize
            cv::Mat img_clone = img.clone();
            draw_track_targets_msg_on_img(img_clone, cur_track_targets);
            m_impl->out_video_writer.write(img_clone);
            RCLCPP_INFO(m_impl->logger, "_process_step(): write frame %d to video", frame_num);

            auto track_targets_frame = std::make_tuple(cur_track_targets, frame);
            {
                auto lock_ptr_track_targets_map = m_impl->sync_track_targets_map.synchronize();
                _add_track_targets_to_buffer(cur_track_targets, frame, *lock_ptr_track_targets_map);
            }
        } else if (frame_num != -1) { // track
            m_impl->ros_track_event_handler->clear();
            m_impl->tracker->track(img, detections, frame_num + 1);
            RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d track", frame_num);
            // put it in std::map<int, std::tuple<MSG_TrackTargets, MSG_Frame>>
            for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_create) {
                cur_track_targets.push_back(track_target_msg);
            }
            for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_associate) {
                cur_track_targets.push_back(track_target_msg);
            }

            // visiualize
            cv::Mat img_clone = img.clone();
            draw_track_targets_msg_on_img(img_clone, cur_track_targets);
            m_impl->out_video_writer.write(img_clone);
            RCLCPP_INFO(m_impl->logger, "_process_step(): write frame %d to video", frame_num);

            for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_closed) {
                cur_track_targets.push_back(track_target_msg);
            }

            // // test log
            RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d track_targets size %d", frame_num, cur_track_targets.size());
            // for (const auto &track_target : cur_track_targets) {
            //     RCLCPP_INFO(m_impl->logger, "_process_step(): track_target %s",
            //                 track_target_msg_to_string(track_target).c_str());
            // }

            auto track_targets_frame = std::make_tuple(cur_track_targets, frame);
            {
                auto lock_ptr_track_targets_map = m_impl->sync_track_targets_map.synchronize();
                _add_track_targets_to_buffer(cur_track_targets, frame, *lock_ptr_track_targets_map);
            }
        } else { // last frame
            m_impl->ros_track_event_handler->clear();
            m_impl->tracker->finish_track();
            RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d finish_track", frame_num);
            // put it in std::map<int, std::tuple<MSG_TrackTargets, MSG_Frame>>
            for (auto &track_target_msg : m_impl->ros_track_event_handler->m_target_closed) {
                cur_track_targets.push_back(track_target_msg);
            }

            // // test log
            RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d track_targets size %d", frame_num, cur_track_targets.size());
            // for (const auto &track_target : cur_track_targets) {
            //     RCLCPP_INFO(m_impl->logger, "_process_step(): track_target %s",
            //                 track_target_msg_to_string(track_target).c_str());
            // }

            auto track_targets_frame = std::make_tuple(cur_track_targets, frame);
            {
                auto lock_ptr_track_targets_map = m_impl->sync_track_targets_map.synchronize();
                _add_track_targets_to_buffer(cur_track_targets, frame, *lock_ptr_track_targets_map);
            }
        }

        if (frame_num == 86) {
            m_impl->out_video_writer.release();
        }

        // remove it from buffer
        {
            auto lock_ptr_detections_map = m_impl->sync_detections_map.synchronize();
            _remove_detections_from_buffer(frame_num, *lock_ptr_detections_map);
        }
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
    // // sort track targets by frame number
    // std::sort(track_targets_task_waiting_.begin(), track_targets_task_waiting_.end(),
    //           [](std::pair<std::tuple<FlowRos2Pipeline::Tracker::Downstream *, int>, std::shared_ptr<FlowRos2Pipeline::Tracker::DSTask_TrackTargets>> &a,
    //              std::pair<std::tuple<FlowRos2Pipeline::Tracker::Downstream *, int>, std::shared_ptr<FlowRos2Pipeline::Tracker::DSTask_TrackTargets>> &b) {
    //               return std::get<1>(a.first) < std::get<1>(b.first);
    //           });

    for (auto &it : track_targets_task_waiting_) {
        auto &task = it.second;
        ACT_AcceptTrackTargets::Goal goal;
        goal.track_targets = task->track_targets;
        goal.frame = task->frame;
        auto ds = task->downstream;
        auto handle = task->downstream->accept_track_targets->async_send_goal(goal, ds->accept_track_targets_options);

        // // test log
        // for (auto &track_target : goal.track_targets) {
        //     RCLCPP_INFO(m_impl->logger, "_send_track_targets_to_downstreams(): send track target %s",
        //                 track_target_msg_to_string(track_target).c_str());
        // }

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
    // // test log
    // for (auto &detection : detections.detections) {
    //     RCLCPP_INFO(m_impl->logger, "_add_detections_to_buffer(): add detection %s",
    //                 detection_msg_to_string(detection).c_str());
    // }
    (*detections_map_ptr)[detections.frame.frame_num] = detections;
}

void Tracker::_add_track_targets_to_buffer(const MSG_TrackTargets &track_targets, const MSG_Frame &frame, Map_TrackTargets *track_targets_map_ptr)
{
    (*track_targets_map_ptr)[frame.frame_num] = std::make_tuple(track_targets, frame);
}

void Tracker::_remove_detections_from_buffer(int frame_number, Map_Detections *detections_map_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_remove_detections_from_buffer(): remove detections with frame_num %d", frame_number);
    // if frame_number is not in buffer, do nothing
    if (detections_map_ptr->find(frame_number) != detections_map_ptr->end()) {
        detections_map_ptr->erase(frame_number);
        RCLCPP_INFO(m_impl->logger, "_remove_detections_from_buffer(): remove detections with frame_num %d SUCCESS", frame_number);
    }
}

void Tracker::_remove_track_targets_from_buffer(int frame_number, Map_TrackTargets *track_targets_map_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_remove_track_targets_from_buffer(): remove track_targets with frame_num %d", frame_number);
    // if frame_number is not in buffer, do nothing
    if (track_targets_map_ptr->find(frame_number) != track_targets_map_ptr->end()) {
        track_targets_map_ptr->erase(frame_number);
        RCLCPP_INFO(m_impl->logger, "_remove_track_targets_from_buffer(): remove track_targets with frame_num %d SUCCESS", frame_number);
    }
}

void ROSTracker::init(const RedoxiTrack::TrackerBasePtr tracker_ptr, const TrackEventHandlerPtr track_event_handler)
{
    m_tracker = tracker_ptr;
    m_track_event_handler = track_event_handler;
}

void ROSTracker::begin_track(const cv::Mat &img, const Tracker::MSG_Detections &detections, int frame_number)
{
    RCLCPP_INFO(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track()");
    std::vector<RedoxiTrack::DetectionPtr> det_bodies;
    std::map<RedoxiTrack::DetectionPtr, std::pair<int, Tracker::MSG_Detection>> det_bodies2msg_detections;
    _msg_dets_to_body_dets(detections, det_bodies, det_bodies2msg_detections);
    RCLCPP_INFO(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track() after _msg_dets_to_body_dets()");

    // // test log
    // for (auto &det_body : det_bodies) {
    //     RCLCPP_INFO(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track() det_body %s",
    //                 psg_detection_to_string(det_body).c_str());
    // }

    m_track_event_handler->clear();
    RCLCPP_INFO(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track() before m_tracker->begin_track()");
    m_tracker->begin_track(img, det_bodies, frame_number);
    RCLCPP_INFO(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track() after m_tracker->begin_track()");
    for (auto &iter : m_track_event_handler->m_det2target_create) {
        auto track_det = iter.first;
        auto track_target = iter.second;
        auto detection_msg = det_bodies2msg_detections[track_det].second;

        // // test log
        // RCLCPP_INFO(rclcpp::get_logger("tracker_node"), "ROSTracker::begin_track(): create track target %s",
        //             psg_track_target_to_string(track_target).c_str());

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

    // // test log
    // for (auto &det_body : det_bodies) {
    //     RCLCPP_INFO(rclcpp::get_logger("tracker_node"), "ROSTracker::track() det_body %s",
    //                 psg_detection_to_string(det_body).c_str());
    // }

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

void ROSTracker::add_event_handler(const ROSTrackEventHandlerPtr &ros_event_handler)
{
    m_ros_track_event_handler_set.insert(ros_event_handler);
}

void ROSTracker::remove_event_handler(const ROSTrackEventHandlerPtr &ros_event_handler)
{
    m_ros_track_event_handler_set.erase(ros_event_handler);
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
    track_target_msg.frame.frame_num = track_target->get_end_frame_number();

    return track_target_msg;
}

} // namespace FlowRos2Pipeline