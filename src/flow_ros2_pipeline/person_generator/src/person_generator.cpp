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
    // RCLCPP_INFO(m_impl->logger, "constraction success!");
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
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
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
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);

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
    // RCLCPP_INFO(m_impl->logger, "m_status_code from %d to %d!", status_before, m_status_code);
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
    // RCLCPP_INFO(m_impl->logger, "Received goal request with psg document %ld", goal->document.frame.frame_num);
    (void)uuid; // not used
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PersonGenerator::_accept_document_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    // RCLCPP_INFO(m_impl->logger, "Received request to cancel goal");
    (void)goal_handle; // not used
    return rclcpp_action::CancelResponse::REJECT;
}

void PersonGenerator::_accept_document_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ACT_AcceptDocument>> goal_handle)
{
    const auto &goal = goal_handle->get_goal();
    const auto &control_msg = goal->control_msg;

    // // if buffer is full, reject the frame
    // if (m_psgdoc_task_waiting.size() >= m_runtime_config->buffer_size || m_frame_task_waiting.size() >= m_runtime_config->buffer_size) {
    //     auto result = std::make_shared<ACT_AcceptDocument::Result>();
    //     result->return_msg = "Buffer is full";
    //     result->return_code = ReturnCode::REJECTED;
    //     goal_handle->abort(result);
    //     return;
    // }

    // ping
    if (control_msg.control_signal == 1) {
        auto result = std::make_shared<ACT_AcceptDocument::Result>();
        result->return_msg = "Ping accepted";
        result->return_code = ReturnCode::SUCCESS;
        goal_handle->succeed(result);
        return;
    }

    // time log
    RCLCPP_INFO(m_impl->logger, "---TIME LOG: framenum %ld node %s type %s time %ld", goal->document.frame.frame_num, "person_generator", "IN", this->now().nanoseconds());

    // cache the document, copy it for modify it
    const auto &document = goal->document;

    // add to memory buffer
    if (document.signal_code == SignalCode::RUN) {
        auto lock_ptr_document_buffer = m_impl->sync_document_buffer.synchronize();
        _add_document_to_buffer(document, *lock_ptr_document_buffer);
        // RCLCPP_INFO(m_impl->logger, "_accept_document_accepted_callback(): Accepted document %ld and add it to buffer", document.frame.frame_num);
    } else {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        _process_document_create_tasks(document, *lock_ptr_psgdoc_task_waiting);
        // RCLCPP_INFO(m_impl->logger, "_accept_document_accepted_callback(): Accepted document %ld and create task", document.frame.frame_num);
    }

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

        // RCLCPP_DEBUG(m_impl->logger, "_process(): frame %ld extracted %ld persons", document.frame.frame_num, v_persons.size());

        // convert to msg
        psg_private_msgs::msg::Persons persons_msg;
        convert_persons_to_msg(v_persons, document.frame, persons_msg);
        // create tasks
        document.persons = persons_msg;
        {
            auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
            _process_document_create_tasks(document, *lock_ptr_psgdoc_task_waiting);
            // RCLCPP_DEBUG(m_impl->logger, "_process(): frame %ld create tasks", document.frame.frame_num);
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
        // RCLCPP_INFO(m_impl->logger, "connecting to pipeline downstream %s", it.first.c_str());

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
            // RCLCPP_INFO(m_impl->logger, "waiting for pipeline action server %s", name.c_str());
            client->wait_for_action_server();
            // RCLCPP_INFO(m_impl->logger, "pipeline action server %s is ready", name.c_str());
        }

        m_pipeline_downstreams[it.first] = ds;
    }
}

bool PersonGenerator::_ping(std::shared_ptr<Downstream> ds)
{
    auto goal_msg = ACT_AcceptDocument::Goal();
    goal_msg.control_msg.control_signal = 1; // ping
    goal_msg.control_msg.control_msg = "ping";

    // opt.goal_response_callback = callback;
    auto res = ds->accept_document->async_send_goal(goal_msg, ds->accept_document_options);

    auto t = (long)m_runtime_config->timeout_ms_send_to_downstream;
    auto wait_result = res.wait_for(std::chrono::milliseconds(t));
    if (wait_result == std::future_status::ready) {
        auto s = res.get()->get_status();
        bool ok = false;

        // downstream accepted?
        ok |= s == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
        ok |= s == rclcpp_action::GoalStatus::STATUS_SUCCEEDED;
        ok |= s == rclcpp_action::GoalStatus::STATUS_EXECUTING;

        if (!ok)
            return false;
    } else
        return false;
    return true;
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
            // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): psgdoc_task_waiting_ push_back framenumber %d", std::get<1>(it.first));
        }
    }

    for (auto &it : psgdoc_task_waiting_) {
        // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): psgdoc_task_waiting_ framenumber %d", std::get<1>(it.first));
        auto &task = it.second;
        auto ds = task->downstream;

        while (true) {
            if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) // not retry need to ping
                if (!_ping(ds))
                    continue;

            ACT_AcceptDocument::Goal goal;
            goal.document = task->document;

            // time log
            RCLCPP_INFO(m_impl->logger, "---TIME LOG: framenum %ld node %s type %s time %ld", goal.document.frame.frame_num, "person_generator", "OUT", this->now().nanoseconds());

            auto handle = task->downstream->accept_document->async_send_goal(goal, ds->accept_document_options);
            // RCLCPP_INFO(m_impl->logger, "_send_document_to_downstreams(): document async_send_goal: %ld", task->document.frame.frame_num);

            // add timeout condition
            auto t = (long)m_runtime_config->timeout_ms_send_to_downstream;
            // RCLCPP_INFO(m_impl->logger, "send goal %ld", task->frame.frame_num);
            auto wait_result = handle.wait_for(std::chrono::milliseconds(t));
            // RCLCPP_INFO(m_impl->logger, "after wait send goal %ld, wait_result is %d", task->frame.frame_num, wait_result);
            if (wait_result == std::future_status::ready) {
                auto task_response = handle.get();
                // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): document async_send_goal: %ld SUCCESS", task->document.frame.frame_num);
                if (task_response != nullptr) {
                    // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): document async_send_goal: %ld task_response is %d",
                    //              task->document.frame.frame_num, task_response->get_status());
                    // accepted or executing
                    if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
                        task_response->get_status() == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
                        // successfully sent, record this
                        task->goal_handle = task_response;
                        {
                            auto lock_ptr_psgdoc_task_doing = m_impl->sync_document_doing_map.synchronize();
                            (**lock_ptr_psgdoc_task_doing)[task->goal_handle] = task;
                        }
                        tasks_to_remove.push_back(it.first);
                        break;
                        // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): STATUS_ACCEPTED tasks_to_remove push_back framenumber %d", std::get<1>(it.first));
                    }

                    // succeed
                    else if (task_response->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED) {
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        break;
                        // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): STATUS_SUCCEEDED tasks_to_remove push_back framenumber %d", std::get<1>(it.first));
                    }
                    // rejected
                    else {
                        if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                            m_psgdoc_task_done.push_back(task);
                            tasks_to_remove.push_back(it.first);
                            break;
                        } else { // retry
                            auto lock_ptr_frame_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                            (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                            continue;
                        }
                    }
                } else {
                    // rejected
                    if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                        m_psgdoc_task_done.push_back(task);
                        tasks_to_remove.push_back(it.first);
                        break;
                    } else { // retry
                        auto lock_ptr_frame_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                        (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                        continue;
                    }
                }
            } else {
                // timeout
                if (!m_runtime_config->send_goal_retry && task->document.frame.signal_code == SignalCode::RUN) { // failed
                    m_psgdoc_task_done.push_back(task);
                    tasks_to_remove.push_back(it.first);
                    break;
                } else { // retry
                    auto lock_ptr_frame_task_waiting = m_impl->sync_document_waiting_map.synchronize();
                    (**lock_ptr_frame_task_waiting)[it.first]->retry_times++;
                    continue;
                }
            }
        }
    }

    // remove all sent tasks
    {
        auto lock_ptr_psgdoc_task_waiting = m_impl->sync_document_waiting_map.synchronize();
        for (auto &it : tasks_to_remove) {
            // RCLCPP_DEBUG(m_impl->logger, "_send_document_to_downstreams(): tasks_to_remove framenumber %d", std::get<1>(it));
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
    this->declare_parameter<int>("buffer_size", 1);
    this->declare_parameter<bool>("send_goal_retry", false);
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