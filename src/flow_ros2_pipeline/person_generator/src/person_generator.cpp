#include <boost/thread/synchronized_value.hpp>

#include <person_generator/_person_generator.hpp>
#include <person_generator/person_generator.hpp>
#include <psg_common/msg_converter.hpp>
#include <rcpputils/asserts.hpp>
#include <vector>

static constexpr auto ROS_ASSERT = rcpputils::assert_true;

using namespace std::chrono_literals;

namespace FlowRos2Pipeline
{
PersonGenerator::PersonGenerator()
    : Node("person_generator_node")
{
    m_impl = std::make_shared<PersonGeneratorImpl>(this);

    _declare_all_parameters();

    // init impl members
    m_impl->sync_document_waiting_map = &m_psgdoc_task_waiting;
    m_impl->sync_document_doing_map = &m_psgdoc_task_doing;

    m_impl->sync_document_buffer = &m_document_buffer;
    RCLCPP_INFO(m_impl->logger, "constraction success!");
}

int PersonGenerator::init(const std::shared_ptr<InitConfig> &config,
                          const std::shared_ptr<RuntimeConfig> &runtime_config)
{
    ROS_ASSERT(m_status_code == NodeStatusCode::BEFORE_INIT && m_status_code != NodeStatusCode::STOPPED,
               "init FAILED! status code is not BEFORE_INIT or STOPPED");

    m_init_config = config;
    m_runtime_config = runtime_config;

    // setup downstreams
    _connect_to_downstreams();

    // create server
    m_act_process_document = rclcpp_action::create_server<ACT_AcceptDocument>(
        this, m_init_config->process_document_action,
        std::bind(&PersonGenerator::_accept_document_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PersonGenerator::_accept_document_cancel_callback, this, std::placeholders::_1),
        std::bind(&PersonGenerator::_accept_document_accepted_callback, this, std::placeholders::_1));

    auto status_before = m_status_code;
    m_status_code = NodeStatusCode::INITIALIZED;
    RCLCPP_INFO(m_impl->logger,
                "m_status_code from %d to %d!",
                status_before, m_status_code);
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PersonGenerator::InitConfig> &PersonGenerator::get_init_config() const
{
    return m_init_config;
}

int PersonGenerator::update_runtime_config(const std::shared_ptr<RuntimeConfig> &config)
{
    ROS_ASSERT(m_status_code != NodeStatusCode::STARTED &&
                   m_status_code != NodeStatusCode::BEFORE_INIT,
               "cannot update_runtime_config");

    m_runtime_config = config;
    return ReturnCode::SUCCESS;
}

const std::shared_ptr<PersonGenerator::RuntimeConfig> &PersonGenerator::get_runtime_config() const
{
    return m_runtime_config;
}


int PersonGenerator::start()
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
                _process();
            }
        });

    return ReturnCode::SUCCESS;
}

int PersonGenerator::stop()
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


int PersonGenerator::get_status_code() const
{
    return m_status_code;
}


rclcpp_action::GoalResponse PersonGenerator::_accept_document_goal_callback(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const ACT_AcceptDocument::Goal> goal)
{
    RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PersonGenerator::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PersonGenerator::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{

    const auto &goal = goal_handle->get_goal();

    // cache the document, copy it for modify it
    const auto &document = goal->document;

    // add to memory buffer
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        _add_document_to_buffer(document, *lock_ptr_document_buffer);
    }

    RCLCPP_INFO(m_impl->logger, "_accept_document_accepted_callback(): Accepted document %ld and add it to buffer", document.frame.frame_num);

    auto result = std::make_shared<ACT_AcceptDocument::Result>();
    result->return_msg = "Document accepted";
    result->return_code = ReturnCode::SUCCESS;
    goal_handle->succeed(result);
}


void PersonGenerator::_process_document_create_tasks(const MSG_PsgDocument &document, Map_Document_Waiting *psgdoc_task_waiting_ptr)
{
    // create tasks of this frame for all downstreams
    for (auto &x : m_pipeline_downstreams) {

        auto task = std::make_shared<DSTask_PsgDocument>();
        task->downstream = x.second;
        task->document = document;
        (*psgdoc_task_waiting_ptr)[std::make_tuple(task->downstream.get(), document.frame.frame_num)] = task;
    }
}


void PersonGenerator::_step()
{
    _send_document_to_downstreams();
}


void PersonGenerator::_process()
{
    std::vector<MSG_PsgDocument> documents_;
    {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        for (auto &it : **lock_ptr_document_buffer) {
            documents_.push_back(it.second);
        }
    }
    // from buffer, extract person
    for (auto &document : documents_) {
        // process document
        std::vector<PassengerFlow::DetectionPtr> v_detections;
        convert_msg_to_detections(document.detections, v_detections);
        // extract person
        auto v_persons = m_person_extractor.extract_persons(v_detections);

        RCLCPP_INFO(m_impl->logger, "_process(): frame %ld extracted %ld persons", document.frame.frame_num, v_persons.size());

        // convert to msg
        psg_private_msgs::msg::Persons persons_msg;
        convert_persons_to_msg(v_persons, persons_msg);
        // create tasks
        document.persons = persons_msg;
        {
            auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
            _process_document_create_tasks(document, *lock_ptr_psgdoc_task_waiting);
        }

        // remove from buffer
        {
            auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
            _remove_document_from_buffer(document.frame.frame_num, *lock_ptr_document_buffer);
        }
    }
}


void PersonGenerator::_connect_to_downstreams()
{
    ROS_ASSERT(m_init_config != nullptr, "m_init_config is nullptr");

    m_pipeline_downstreams.clear();

    for (auto it : m_init_config->pipeline_downstreams) {
        auto ds = std::make_shared<Downstream>();
        RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

        // 创建pipeline downstream
        {
            std::string name = it.second.accept_document_action;
            auto client = rclcpp_action::create_client<ACT_AcceptDocument>(this, name);

            ds->accept_document = client;
            // ds->accept_document_options.goal_response_callback =
            //         std::bind(&PersonGenerator::process_document_goal_response_callback, this, std::placeholders::_1);
            // ds->accept_document_options.feedback_callback =
            //         std::bind(&PersonGenerator::process_document_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            // ds->accept_document_options.result_callback =
            //         std::bind(&PersonGenerator::process_document_result_callback, this, std::placeholders::_1);

            // wait until the action server is ready
            RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_pipeline_downstreams[it.first] = ds;
    }
}

void PersonGenerator::_send_document_to_downstreams()
{
    // initiate all waiting tasks
    std::vector<Map_Document_Waiting::key_type> tasks_to_remove;

    std::vector<decltype(m_psgdoc_task_waiting)::value_type> psgdoc_task_waiting_;
    {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();

        for (auto const &it : (**lock_ptr_psgdoc_task_waiting)) {
            psgdoc_task_waiting_.push_back(it);
        }
    }

    for (auto &it : psgdoc_task_waiting_) {
        auto &task = it.second;
        ACT_AcceptDocument::Goal goal;
        goal.document = task->document;
        auto ds = task->downstream;
        auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);
        RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): document async_send_goal: %ld", task->document.frame.frame_num);

        // FIXME: add timeout condition
        auto task_response = handle.get();
        RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): document async_send_goal: %ld SUCCESS", task->document.frame.frame_num);
        if (task_response != nullptr) {
            // accepted
            if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
                // successfully sent, record this
                task->goal_handle = task_response;
                {
                    auto lock_ptr_psgdoc_task_doing = m_impl->sync_document_doing_map.synchronize();
                    (**lock_ptr_psgdoc_task_doing)[task->goal_handle] = task;
                }
                tasks_to_remove.push_back(it.first);
            }

            // succeed
            else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                // task->status = DSTask_PSGDocument::TASK_DONE;
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
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            (*lock_ptr_psgdoc_task_waiting)->erase(it);
        }
    }

    {
        auto lock_ptr_psgdoc_task_doing = m_impl->sync_document_doing_map.synchronize();
        // for on-going tasks, if it is done, remove it
        if (!(*lock_ptr_psgdoc_task_doing)->empty()) {
            std::vector<GoalHandle_PsgDocument> tasks_to_remove;
            for (auto &it : (**lock_ptr_psgdoc_task_doing)) {
                auto &task_response = it.first;
                if (task_response) {
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        tasks_to_remove.push_back(it.first);
                    }
                }
            }

            for (auto &it : tasks_to_remove) {
                (*lock_ptr_psgdoc_task_doing)->erase(it);
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


void PersonGenerator::_declare_all_parameters()
{
    this->declare_parameter<std::string>("process_document_action", "");
    this->declare_parameter<double>("step_interval_ms", -1);
    this->declare_parameter<double>("timeout_ms_send_to_downstream", -1);
}


void PersonGenerator::_add_document_to_buffer(const MSG_PsgDocument &document, std::map<int, MSG_PsgDocument> *document_buffer_ptr)
{
    (*document_buffer_ptr)[document.frame.frame_num] = document;
}

void PersonGenerator::_remove_document_from_buffer(int frame_number, std::map<int, MSG_PsgDocument> *document_buffer_ptr)
{
    // if frame_number is not in buffer, do nothing
    if (document_buffer_ptr->find(frame_number) != document_buffer_ptr->end()) {
        document_buffer_ptr->erase(frame_number);
    }
}
} // namespace FlowRos2Pipeline