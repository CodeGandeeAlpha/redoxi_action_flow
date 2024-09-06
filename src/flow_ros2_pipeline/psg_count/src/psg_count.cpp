#include <rclcpp/utilities.hpp>

#include <rcpputils/asserts.hpp>
#include <psg_count/_psg_count.hpp>
#include <psg_count/psg_count.hpp>
#include <psg_common/msg_converter.hpp>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
cv::Scalar _get_color(const int id)
{
    int idx = id * 3;
    cv::Scalar color((37 * idx) % 255, (17 * idx) % 255, (29 * idx) % 255);
    return color;
}

void draw_person(const PassengerFlow::CameraModelPtr& cam, const PassengerFlow::GroundPtr& ground,
            cv::Mat& img, const PassengerFlow::PersonPtr& person){
    const std::string person_id = std::to_string(person->get_person_id());
    auto c = _get_color(person->get_person_id());
    if(person->body()){
        cv::rectangle(img, person->body()->get_bbox(), c, 2,1,0);

        cv::Point2i text_origin(person->body()->get_bbox().x, person->body()->get_bbox().y);
        cv::putText(img, person_id, text_origin, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,255),2);
    }
    if(person->head()){
        cv::rectangle(img, person->head()->get_bbox(), c, 2,1,0);
    }
    if(person->face()){
        cv::rectangle(img, person->face()->get_bbox(), c, 2,1,0);
    }

    PassengerFlow::POINT3 foot_position_in_world;
    bool position_valid;
    person->get_foot_position(&foot_position_in_world, &position_valid);
    if(position_valid) {
        PassengerFlow::fVECTOR_3 foot_position_vec{foot_position_in_world.x, foot_position_in_world.y,
                                                    foot_position_in_world.z};
        auto foot_position_uv = cam->project_points(foot_position_vec);
        int u = foot_position_uv[0], v = foot_position_uv[1];
        cv::Point2i foot_position_in_img{u, v};
        // cv::putText(img, person_id, foot_position_in_img, cv::FONT_HERSHEY_SIMPLEX, 1, c, 1);
        cv::circle(img, foot_position_in_img, 2, c, 4);
    }

}

void draw_region_points(cv::Mat& img, const cv::Scalar& region_color, const std::string &region_name,
                    const std::vector<PassengerFlow::POINT> &region_points,
                        const PassengerFlow::GroundPtr &ground, const PassengerFlow::CameraModelPtr &camera) {
    std::vector<cv::Point2i> region_points_on_img;
    int center_u=0, center_v=0;
    for(auto& point : region_points){
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
    PassengerFlow::POINT pre_point=region_points_on_img[0], next_point;
    for(int i = 1; i<region_points_on_img.size();++i){
        next_point = region_points_on_img[i];
        cv::line(img, pre_point, next_point, region_color);
        pre_point = next_point;
    }
    next_point = region_points_on_img[0];
    cv::line(img, pre_point, next_point, region_color);
    cv::putText(img, region_name, cv::Point2i(center_u, center_v), cv::FONT_HERSHEY_SIMPLEX, 1, region_color,2);
}

void draw_event_zone(cv::Mat& img, const std::map<std::string, PassengerFlow::EventZonePtr> &event_zones,
                const PassengerFlow::GroundPtr &ground, const PassengerFlow::CameraModelPtr &camera){
    int rand_seed = 0;
    for(auto& iter:event_zones){
        srand(rand_seed);
        rand_seed +=10;
        int r = rand() % 255;
        int b = rand() % 255;
        cv::Scalar event_zone_color(b, 0, r);
        auto event_regions = iter.second->get_region_points();
        for(auto&region:event_regions){
            draw_region_points(img, event_zone_color, region.first, region.second, ground, camera);
        }
    }
}

PSGCount::PSGCount()
    : Node("psg_count_node")
{
    m_impl = std::make_shared<PSGCountImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    m_impl->sync_document_doing_map = &m_psgdoc_task_doing;
    m_impl->sync_documents_map = &m_documents_buffer;

    RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int PSGCount::init(const std::shared_ptr<InitConfig> &config,
                    const std::shared_ptr<RuntimeConfig> &runtime_config)
{
    ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::STOPPED,
               "init FAILED! status code is not BEFORE_INIT or STOPPED");

    m_init_config = config;
    m_runtime_config = runtime_config;

    // connect to v6d
    m_impl->v6d_client = create_v6d_client();

    // setup analyzers
    m_impl->spatial_analyzer = std::make_shared<PassengerFlow::SingleGroundSpatialAnalyzer3>();
    m_impl->trajectory_analyzer = std::make_shared<PassengerFlow::TrajectoryAnalyzer>();
    auto param = m_impl->parse_config_file(m_init_config->passengerflow_config_path);
    m_impl->camera = std::make_shared<PassengerFlow::CameraModel>();
    m_impl->ground = std::make_shared<PassengerFlow::Ground>();
    m_impl->scene = std::make_shared<PassengerFlow::Scene>();
    m_impl->set_scene(param, m_impl->scene, m_impl->camera, m_impl->ground, m_impl->event_zones);

    m_impl->spatial_analyzer->set_scene(m_impl->scene);
    m_impl->spatial_analyzer->set_ground(m_impl->ground);

    for (auto &iter: m_impl->event_zones) {
        m_impl->trajectory_analyzer->set_event_zone(iter.first, iter.second);
    }

    // // setup downstreams
    // _connect_to_downstreams();

    // create server
    m_act_process_document = rclcpp_action::create_server<ACT_AcceptDocument>(
        this, m_init_config->process_document_action,
        std::bind(&PSGCount::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PSGCount::_accept_document_cancel_callback, this, std::placeholders::_1),
        std::bind(&PSGCount::_accept_document_accepted_callback, this, std::placeholders::_1));

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PSGCount::InitConfig> &PSGCount::get_init_config() const
{
    return m_init_config;
}

int PSGCount::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PSGCount::RuntimeConfig> &PSGCount::get_runtime_config() const
{
    return m_runtime_config;
}


int PSGCount::start()
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

    m_impl->process_thread = std::make_shared<std::thread>(
        [this]() {
            while (rclcpp::ok() && m_impl->step_running) {
                _process_step();
            }
        });

    return ReturnCode::SUCCESS;
}

int PSGCount::stop()
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

    // closing, release v6d client
    m_impl->v6d_client->Disconnect();
    m_impl->v6d_client = nullptr;

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::STOPPED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);
    return ReturnCode::SUCCESS;
}


int PSGCount::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse PSGCount::_accept_document_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDocument::Goal> goal)
{
    RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PSGCount::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PSGCount::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the document, copy it for modify it
    auto document = goal->document;

    // add it to documents buffer
    {
        auto lock_ptr_documents_map = m_impl->sync_documents_map.synchronize();
        // if is empty frame, mark it as INT_MAX
        if (document.frame.frame_num == -1) {
            document.frame.frame_num = INT_MAX;
        }
        _add_document_to_buffer(document, *lock_ptr_documents_map);
    }

    RCLCPP_INFO(m_impl->logger, "_accept_document_accepted_callback(): Accepted frame_number %ld and add it to buffer", document.frame.frame_num);

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);

    RCLCPP_INFO(m_impl->logger, "_accept_document_accepted_callback(): return client success in frame_number %ld", document.frame.frame_num);
}


void PSGCount::_process_document_create_tasks(const MSG_PsgDocument &document, Map_Document_Waiting *psgdoc_task_waiting_ptr)
{
    // create tasks of this frame for all downstreams
    for (auto &x : m_pipeline_downstreams) {

        auto task = std::make_shared<DSTask_PsgDocument>();
        task->downstream = x.second;
        task->document = document;
        (*psgdoc_task_waiting_ptr)[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
    }
}

void PSGCount::_add_document_to_buffer(const MSG_PsgDocument &document, Map_Documents *document_buffer_ptr)
{
    (*document_buffer_ptr)[document.frame.frame_num] = document;
}

void PSGCount::_remove_document_from_buffer(int frame_number, Map_Documents *document_buffer_ptr)
{
    RCLCPP_INFO(m_impl->logger, "_remove_document_from_buffer(): remove document with frame_num %d", frame_number);
    // if frame_number is not in buffer, do nothing
    if (document_buffer_ptr->find(frame_number) != document_buffer_ptr->end()) {
        document_buffer_ptr->erase(frame_number);
        RCLCPP_INFO(m_impl->logger, "_remove_document_from_buffer(): remove document with frame_num %d SUCCESS", frame_number);
    }
}

void PSGCount::_step()
{
    // _send_document_to_downstreams();
}

void PSGCount::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

    m_pipeline_downstreams.clear();

    for (auto it : m_init_config->pipeline_downstreams) {
        auto ds = std::make_shared<DownstreamPipeline>();
        RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

        // create pipeline downstream
        {
            std::string name = it.second.accept_document_action;
            auto client = rclcpp_action::create_client<ACT_AcceptDocument>(this, name);

            ds->accept_document = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&PSGCount::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&PSGCount::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&PSGCount::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_pipeline_downstreams[it.first] = ds;
    }
}

void PSGCount::_send_document_to_downstreams()
{
    std::vector<decltype(m_psgdoc_task_waiting)::value_type> document_task_waiting_;
    {
        auto lock_ptr_psg_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();

        for (auto const &it : (**lock_ptr_psg_document_task_waiting)) {
            document_task_waiting_.push_back(it);
        }
    }

    std::vector<Map_Document_Waiting::key_type> tasks_to_remove;

    for (auto &it : document_task_waiting_) {
        auto &task = it.second;
        ACT_AcceptDocument::Goal goal;
        goal.document = task->document;

        auto ds = task->downstream;
        auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);

        RCLCPP_INFO(m_impl->logger, "[Request Send]framenum: %ld document", goal.document.frame.frame_num);

        // FIXME: add timeout condition
        auto task_response = handle.get();
        RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): frame async_send_goal: %ld SUCCESS", task->document.frame.frame_num);
        if (task_response != nullptr) {
            RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): async_send_goal: %ld task_response is %d",
                        task->document.frame.frame_num, task_response->get_status());
            // accepted or executing
            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
                // successfully sent, record this
                task->goal_handle = task_response;
                {
                    auto lock_ptr_psg_document_task_doing = m_impl->sync_document_doing_map.synchronize();
                    (**lock_ptr_psg_document_task_doing)[task->goal_handle] = task;
                }
                tasks_to_remove.push_back(it.first);
                RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): STATUS_ACCEPTED tasks_to_remove push_back framenumber %ld", task->document.frame.frame_num);
            }

            // succeed
            else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                m_psgdoc_task_done.push_back(task);
                tasks_to_remove.push_back(it.first);
                RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): STATUS_SUCCEEDED tasks_to_remove push_back framenumber %ld", task->document.frame.frame_num);
            }
        }

        // FIXME: what if failed to send many times?
        // you need to terminate a frame, remove it from memory registry
    }

    // remove all sent tasks
    {
        auto lock_ptr_psg_document_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): tasks_to_remove framenumber %d", std::get<1>(it));
            (*lock_ptr_psg_document_task_waiting)->erase(it);
        }
    }

    {
        auto lock_ptr_psg_document_task_doing = m_impl->sync_document_doing_map.synchronize();
        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_psg_document_task_doing)->empty()) {
            std::vector<GoalHandle_PsgDocument> tasks_to_remove;
            for (auto &it : (**lock_ptr_psg_document_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_psgdoc_task_done.push_back(it.second);
                        tasks_to_remove.push_back(it.first);
                        RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): STATUS_SUCCEEDED tasks_to_remove push_back framenumber %ld", it.second->document.frame.frame_num);
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_psg_document_task_doing)->erase(it);
            }
        }
    }

    // for all done tasks, remove them from memory
    m_psgdoc_task_done.clear();
}


void PSGCount::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<std::string>("passengerflow_config_path", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
}


void PSGCount::_process_step()
{
    std::vector<std::pair<int, MSG_PsgDocument>> documents_;
    {
        auto lock_ptr_documents_map = m_impl->sync_documents_map.synchronize();

        for (auto &it : **lock_ptr_documents_map) {
            auto &frame_num = it.first;
            if (frame_num == m_waiting_frame_number) {
                m_waiting_frame_number++;
                documents_.push_back(it);
                RCLCPP_INFO(m_impl->logger, "_process_step(): framenum %d document push_back to documents_", it.first);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(m_runtime_config->step_interval_ms)));
                break;
            }
        }
    }
    // process all documents in buffer
    for (auto &it : documents_) {
        auto &frame_num = it.first;
        auto &document = it.second;

        auto &trajectories = document.trajectories;
        auto &frame = document.frame;

        // count people
        // spatial analysis
        std::vector<PassengerFlow::PersonTrajectory> v_trajs;
        convert_msg_to_trajs(trajectories, v_trajs);
        for (auto &traj: v_trajs) {
            for (auto &person: traj.m_person_list) {
                person->set_ground(m_impl->ground);
                person->set_camera(m_impl->camera);
                if (person->head()) {
                    dynamic_cast<PassengerFlow::Detection*>(person->head().get())->set_camera(m_impl->camera);
                }
            }
        }
        m_impl->spatial_analyzer->process_inplace(v_trajs);

        // get frame from shared memory
        auto tensor = get_tensor_by_v6d_id(frame.cache.id_int, m_impl->v6d_client);
        auto img = from_v6d_tensor_to_cvmat(tensor);
        auto draw_img = img.clone();
        // draw event zones
        draw_event_zone(draw_img, m_impl->event_zones, m_impl->ground, m_impl->camera);

        // trajectory analysis
        std::vector<PassengerFlow::TrajectoryEvent> v_traj_event;
        MSG_TrajectoryEvents msg_traj_events;

        for (auto &traj: v_trajs) {
            auto events = m_impl->trajectory_analyzer->process(traj);

            // test log
            for (auto &person: traj.m_person_list) {
                if (person->body()) {
                    PassengerFlow::POINT3 foot_position;
                    bool position_valid;
                    person->get_foot_position(&foot_position, &position_valid);
                    RCLCPP_INFO(m_impl->logger, "_process_step(): person id: %d, person height: %lf, foot position: (%lf, %lf, %lf)",
                                person->get_person_id(), person->get_body_height().m_body_height, foot_position.x, foot_position.y, foot_position.z);
                }

                // draw person test
                draw_person(m_impl->camera, m_impl->ground, draw_img, person);
            }

            for (auto &iter: events) {
                for (auto &event: iter.second) {
                    RCLCPP_INFO(m_impl->logger, "_process_step(): event id: %d, id: %d, event type: %d, start time: %f, end time: %f, \
                                event info: %s, speed_x: %f, speed_y: %f", event.m_person_id, event.m_person_id, event.m_event_type, event.m_start_time,
                                 event.m_end_time, event.m_matched_trajectory.c_str(), event.m_speed.x, event.m_speed.y);
                    v_traj_event.push_back(event);
                }
            }
        }

        // test visualization output
        cv::imwrite("/mnt/chengxiao/traj_framenum_" + std::to_string(frame.frame_num) + "_out.jpg", draw_img);

        convert_events_to_msg(v_traj_event, msg_traj_events);
        document.trajectory_events = msg_traj_events;

        // create tasks for all downstreams
        {
            auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
            _process_document_create_tasks(document, *lock_ptr_psgdoc_task_waiting);
        }
        // remove it from buffer
        {
            auto lock_ptr_documents_map = m_impl->sync_documents_map.synchronize();
            _remove_document_from_buffer(frame_num, *lock_ptr_documents_map);
        }
    }
}
} // namespace FlowRos2Pipeline