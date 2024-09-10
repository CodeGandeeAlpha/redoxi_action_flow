#include <boost/thread/lock_algorithms.hpp>
#include <psg_common/psg_common.hpp>
#include <rcpputils/asserts.hpp>
#include <tracker/_pipeline_out.hpp>
#include <tracker/pipeline_out.hpp>

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

std::string track_target_msg_to_string(const TrackerOut::MSG_TrackTarget &track_target)
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

std::string person_msg_to_string(const TrackerOut::MSG_Person &person)
{
    std::string str = "Person: {\n";
    str += "frame_num: " + std::to_string(person.frame.frame_num) + "\n";
    str += "uuid: " + uuid_to_string(person.uuid.uuid) + "\n";
    str += "track_id: " + std::to_string(person.track_id) + "\n";
    str += "body_bbox: " + bbox_msg_to_string(person.body.bbox) + "\n";
    str += "face_bbox: " + bbox_msg_to_string(person.face.bbox) + "\n";
    str += "head_bbox: " + bbox_msg_to_string(person.head.bbox) + "\n";
    str += "}";

    return str;
}


TrackerOut::TrackerOut()
    : Node("tracker_out_node")
{
    m_impl = std::make_shared<TrackerOutImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    m_impl->sync_document_doing_map = &m_psgdoc_task_doing;

    m_impl->sync_document_buffer = &m_document_buffer;
    m_impl->sync_track_targets_buffer = &m_track_targets_buffer;
    m_impl->sync_person_buffer = &m_person_buffer;

    RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int TrackerOut::init(const std::shared_ptr<InitConfig> &config,
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

    // create process document server
    m_act_process_document = rclcpp_action::create_server<ACT_AcceptDocument>(
        this, m_init_config->process_document_action,
        std::bind(&TrackerOut::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TrackerOut::_accept_document_cancel_callback, this, std::placeholders::_1),
        std::bind(&TrackerOut::_accept_document_accepted_callback, this, std::placeholders::_1));

    // create process track_targets server
    // std::string process_detections_action = this->get_parameter(m_init_config->process_detections_action).as_string();
    m_act_process_track_targets = rclcpp_action::create_server<ACT_AcceptTrackTargets>(
        this, m_init_config->process_track_targets_action,
        std::bind(&TrackerOut::_accept_track_targets_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TrackerOut::_accept_track_targets_cancel_callback, this, std::placeholders::_1),
        std::bind(&TrackerOut::_accept_track_targets_accepted_callback, this, std::placeholders::_1));

    // setup downstreams
    _connect_to_downstreams();

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<TrackerOut::InitConfig> &TrackerOut::get_init_config() const
{
    return m_init_config;
}

int TrackerOut::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<TrackerOut::RuntimeConfig> &TrackerOut::get_runtime_config() const
{
    return m_runtime_config;
}


int TrackerOut::start()
{
    // the node must be opened
    ROS_ASSERT(m_status_code == NodeStatusCode::INITIALIZED,
               "cannot start because status code is not INITIALIZED");

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

    return ReturnCode::SUCCESS;
}

int TrackerOut::stop()
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

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STOPPED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);

    m_status_code = NodeStatusCode::STOPPED;
    return ReturnCode::SUCCESS;
}


int TrackerOut::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse TrackerOut::_accept_document_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDocument::Goal> goal)
{
    RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrackerOut::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void TrackerOut::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the document
    auto document = goal->document;

    // add to buffer
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        _add_document_to_buffer(document, *lock_ptr_document_buffer);
    }

    // add person to buffer
    {
        auto lock_ptr_person_buffer = m_impl->sync_person_buffer.synchronize();
        for (const auto &person : document.persons.persons) {
            // // test log
            // RCLCPP_INFO(m_impl->logger, "_accept_document_accepted_callback(): Accepted person %s",
            //             person_msg_to_string(person).c_str());
            // add to buffer
            _add_person_to_buffer(person, *lock_ptr_person_buffer);
        }
    }

    RCLCPP_INFO(m_impl->logger, "_accept_document_accepted_callback(): Accepted document %ld with UUID %s and add it to buffer",
                document.frame.frame_num, uuid_to_string(document.detections_uuid.uuid).c_str());

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}

rclcpp_action::GoalResponse TrackerOut::_accept_track_targets_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptTrackTargets::Goal> goal)
{
    RCLCPP_INFO(m_impl->logger, "_accept_track_targets_goal_callback(): Received goal request with frame_num %ld", goal->frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrackerOut::_accept_track_targets_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptTrackTargets>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "_accept_track_targets_cancel_callback(): Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void TrackerOut::_accept_track_targets_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptTrackTargets>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the detections
    const auto &track_targets = goal->track_targets;
    auto frame = goal->frame;

    // test log
    RCLCPP_INFO(m_impl->logger, "_accept_track_targets_accepted_callback(): Accepted frame %ld with %ld track_targets", frame.frame_num, track_targets.size());
    for (const auto &track_target : track_targets) {
        RCLCPP_DEBUG(m_impl->logger, "_accept_track_targets_accepted_callback(): Accepted track_target %s",
                    track_target_msg_to_string(track_target).c_str());
    }

    // add to buffer
    {
        auto lock_ptr_track_targets_buffer = m_impl->sync_track_targets_buffer.synchronize();
        _add_track_targets_to_buffer(track_targets, frame.frame_num, *lock_ptr_track_targets_buffer);
    }

    RCLCPP_INFO(m_impl->logger, "_accept_track_targets_accepted_callback(): Accepted frame_number %ld and add it to buffer", frame.frame_num);

    auto result = std::make_shared<ACT_AcceptTrackTargets::Result>();
    result->return_msg = "TrackTargets accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}


void TrackerOut::_process_document_create_tasks(MSG_PsgDocument &document,
                                                TrackerOut::Map_Document_Waiting *document_waiting_map_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_process_document_create_tasks(): create tasks for document %ld", document.frame.frame_num);
    // create tasks of this frame for all downstreams
    for (auto &x : m_downstreams) {
        auto task = std::make_shared<DSTask_PsgDocument>();
        task->downstream = x.second;
        task->document = document;
        (*document_waiting_map_ptr)[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
    }
}


void TrackerOut::_step()
{
    _get_closed_trajectory();
    _send_document_to_downstreams();
}

void TrackerOut::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

    m_downstreams.clear();

    for (auto it : m_init_config->downstreams) {
        auto ds = std::make_shared<Downstream>();
        RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

        // 创建downstream
        {
            std::string name = it.second.accept_document_action;
            auto client = rclcpp_action::create_client<ACT_AcceptDocument>(this, name);

            ds->accept_document = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&TrackerOut::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&TrackerOut::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&TrackerOut::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_downstreams[it.first] = ds;
    }
}

void TrackerOut::_send_document_to_downstreams()
{
    std::vector<Map_Document_Waiting::key_type> tasks_to_remove;
    std::vector<decltype(m_psgdoc_task_waiting)::value_type> psgdoc_task_waiting_;
    {
        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();

        for (auto const &it : (**lock_ptr_document_task_waiting)) {
            psgdoc_task_waiting_.push_back(it);
        }
    }

    for (auto &it : psgdoc_task_waiting_) {
        auto &task = it.second;
        ACT_AcceptDocument::Goal goal;
        goal.document = task->document;
        auto ds = task->downstream;
        auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);

        // FIXME: add timeout condition
        auto task_response = handle.get();
        if (task_response != nullptr) {
            // accepted
            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
                // successfully sent, record this
                task->goal_handle = task_response;
                // task->status = DSTask_PSGDocument::TASK_SENT;
                {
                    auto lock_ptr_document_task_doing = m_impl->sync_document_doing_map.synchronize();
                    (**lock_ptr_document_task_doing)[task->goal_handle] = task;
                }
                tasks_to_remove.push_back(it.first);
            }

            // succeed
            else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                // task->status = DSTask_PSGDocument::TASK_DONE;
                m_psgdoc_task_done.push_back(task);
                tasks_to_remove.push_back(it.first);
            }
        }
        // else {
        //     // rejected
        //     task->status = DSTask_PSGDocument::TASK_FAILED;
        // }

        // FIXME: what if failed to send many times?
        // you need to terminate a frame, remove it from memory registry
    }

    // remove all sent tasks
    {
        auto lock_ptr_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_document_task_waiting)->erase(it);
        }
    }

    {
        auto lock_ptr_document_task_doing = m_impl->sync_document_doing_map.synchronize();
        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_document_task_doing)->empty()) {
            std::vector<GoalHandle_PsgDocument> tasks_to_remove;
            for (auto &it : (**lock_ptr_document_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_psgdoc_task_done.push_back(it.second);
                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_document_task_doing)->erase(it);
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
    m_psgdoc_task_done.clear();
}


void TrackerOut::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<std::string>("process_track_targets_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
}


void TrackerOut::_add_document_to_buffer(MSG_PsgDocument &document, std::map<int, TrackerOut::MSG_PsgDocument> *document_buffer_ptr)
{
    (*document_buffer_ptr)[document.frame.frame_num] = document;
}

void TrackerOut::_add_track_targets_to_buffer(const MSG_TrackTargets &track_targets, const int frame_number, std::map<int, TrackerOut::MSG_TrackTargets> *track_targets_buffer_ptr)
{
    (*track_targets_buffer_ptr)[frame_number] = track_targets;
}

void TrackerOut::_add_person_to_buffer(const MSG_Person &person, std::map<TrackerOut::UUID, TrackerOut::MSG_Person> *person_buffer_ptr)
{
    (*person_buffer_ptr)[person.uuid.uuid] = person;
}

void TrackerOut::_remove_document_from_buffer(int frame_number, std::map<int, TrackerOut::MSG_PsgDocument> *document_buffer_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_remove_document_from_buffer(): remove document with frame_num %d", frame_number);
    // if frame_number is not in buffer, do nothing
    if (document_buffer_ptr->find(frame_number) != document_buffer_ptr->end()) {
        document_buffer_ptr->erase(frame_number);
        RCLCPP_INFO(m_impl->logger, "_remove_document_from_buffer(): remove document with frame_num %d SUCCESS", frame_number);
    }
}

void TrackerOut::_remove_track_targets_from_buffer(int frame_number, std::map<int, TrackerOut::MSG_TrackTargets> *track_targets_buffer_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_remove_track_targets_from_buffer(): remove track_targets with frame_num %d", frame_number);
    // if frame_number is not in buffer, do nothing
    if (track_targets_buffer_ptr->find(frame_number) != track_targets_buffer_ptr->end()) {
        track_targets_buffer_ptr->erase(frame_number);
        RCLCPP_INFO(m_impl->logger, "_remove_track_targets_from_buffer(): remove track_targets with frame_num %d SUCCESS", frame_number);
    }
}

void TrackerOut::_remove_person_from_buffer(const UUID &uuid, std::map<UUID, MSG_Person> *person_buffer_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_remove_person_from_buffer(): remove person with uuid %s", uuid_to_string(uuid).c_str());
    // if uuid is not in buffer, do nothing
    if (person_buffer_ptr->find(uuid) != person_buffer_ptr->end()) {
        person_buffer_ptr->erase(uuid);
        RCLCPP_INFO(m_impl->logger, "_remove_person_from_buffer(): remove person with uuid %s SUCCESS", uuid_to_string(uuid).c_str());
    }
}

void TrackerOut::_get_closed_trajectory()
{
    std::vector<decltype(m_track_targets_buffer)::value_type> track_targets_buffer_;

    {
        auto lock_ptr_track_targets_buffer = m_impl->sync_track_targets_buffer.synchronize();
        for (auto const &it : (**lock_ptr_track_targets_buffer)) {
            track_targets_buffer_.push_back(it);
        }
    }

    for (auto &it : track_targets_buffer_) {
        auto &track_targets = it.second;
        const auto frame_num = it.first;
        MSG_PsgDocument document;
        {
            auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
            document = (**lock_ptr_document_buffer)[frame_num];
        }

        for (auto &track_target : track_targets) {
            // // test log
            // RCLCPP_INFO(m_impl->logger, "_get_closed_trajectory(): track_target %s", track_target_msg_to_string(track_target).c_str());

            // copy track_target info to person
            {
                auto lock_ptr_person_buffer = m_impl->sync_person_buffer.synchronize();
                if ((*lock_ptr_person_buffer)->find(track_target.uuid.uuid) != (*lock_ptr_person_buffer)->end()) {
                    auto &person = (**lock_ptr_person_buffer)[track_target.uuid.uuid];
                    person.track_id = track_target.track_id;
                }
            }

            // if track_target is new, create a new trajectory
            if (track_target.track_status == RedoxiTrack::TrackPathStateBitmask::New) {
                m_closed_trajectory_buffer[track_target.track_id] = std::vector<UUID>();
                m_closed_trajectory_buffer[track_target.track_id].push_back(track_target.uuid.uuid);
                RCLCPP_DEBUG(m_impl->logger, "_get_closed_trajectory(): create new trajectory with track_id %ld", track_target.track_id);
            }
            // if track_target is open, add it to trajectory
            else if (track_target.track_status == RedoxiTrack::TrackPathStateBitmask::Open) {
                m_closed_trajectory_buffer[track_target.track_id].push_back(track_target.uuid.uuid);
                RCLCPP_DEBUG(m_impl->logger, "_get_closed_trajectory(): add track_target to trajectory with track_id %d and frame number %ld", track_target.track_id, frame_num);
            }
            // if track_target is close, get trajectory and remove it from buffer
            else if (track_target.track_status == RedoxiTrack::TrackPathStateBitmask::Close) {
                // get closed trajectory uuids
                auto closed_trajectory_uuids = m_closed_trajectory_buffer[track_target.track_id];
                // remove closed trajectory from buffer
                m_closed_trajectory_buffer.erase(track_target.track_id);
                // get closed trajectory
                MSG_Trajectory closed_trajectory;
                closed_trajectory.track_id = track_target.track_id;
                {
                    auto lock_ptr_person_buffer = m_impl->sync_person_buffer.synchronize();
                    for (auto &uuid : closed_trajectory_uuids) {
                        closed_trajectory.persons.push_back((**lock_ptr_person_buffer)[uuid]);
                        _remove_person_from_buffer(uuid, *lock_ptr_person_buffer);
                    }
                }
                // put closed trajectory to document
                document.trajectories.person_trajectories.push_back(closed_trajectory);

                // test log
                RCLCPP_DEBUG(m_impl->logger, "_get_closed_trajectory(): get closed trajectory with track_id %ld in frame_number %ld", track_target.track_id, frame_num);
                RCLCPP_DEBUG(m_impl->logger, "_get_closed_trajectory(): total closed trajectory %ld", document.trajectories.person_trajectories.size());
                for (auto &traj : document.trajectories.person_trajectories) {
                    RCLCPP_DEBUG(m_impl->logger, "_get_closed_trajectory(): closed trajectory %ld", traj.track_id);
                    for (auto &person : traj.persons) {
                        RCLCPP_DEBUG(m_impl->logger, "_get_closed_trajectory(): person %s", person_msg_to_string(person).c_str());
                    }
                }
            } else
                continue;
        }

        // create task for this frame
        {
            auto lock_ptr_document_waiting_map = m_impl->sync_document_waiting_map.synchronize();
            _process_document_create_tasks(document, *lock_ptr_document_waiting_map);
        }

        // remove document from buffer
        {
            auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
            _remove_document_from_buffer(frame_num, *lock_ptr_document_buffer);
        }

        // remove track_targets from buffer
        {
            auto lock_ptr_track_targets_buffer = m_impl->sync_track_targets_buffer.synchronize();
            _remove_track_targets_from_buffer(frame_num, *lock_ptr_track_targets_buffer);
        }
    }
}

} // namespace FlowRos2Pipeline