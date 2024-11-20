#include <chrono>
#include <map>
#include <yolo8_body_pose_detector/Yolo8BodyPoseDetector.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>
#include <boost/thread/synchronized_value.hpp>

namespace redoxi_works::model_nodes
{

struct Yolo8BodyPoseDetector::Impl {
    tbb::task_group inference_task_group;
    tbb::concurrent_queue<InferenceResource_t> inference_resources;
    boost::synchronized_value<std::map<GoalUUID_t, InferenceResource_t>> goal_to_inference_resource;
};

Yolo8BodyPoseDetector::Yolo8BodyPoseDetector(const std::string &node_name,
                                             const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options)
{
    auto ret = declare_default_parameters_for_node(this);
    if (ret != 0) {
        RDX_RAISE_ERROR("Failed to declare default parameters for node {}", node_name);
    }

    auto node = this;
    m_json_params = RDX_GET_JSON_PARAM_FROM_NODE(node);

    m_impl = std::make_shared<Impl>();
}

Yolo8BodyPoseDetector::~Yolo8BodyPoseDetector() noexcept
{
    // do not call stop() here, because it will NOT call the subclass's stop()

    // let all inference tasks finish
    m_impl->inference_task_group.wait();
}

int Yolo8BodyPoseDetector::init(std::shared_ptr<InitConfig_t> init_config,
                                std::shared_ptr<RuntimeConfig_t> runtime_config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "initializing");

    _update_init_config(init_config);
    _update_runtime_config(runtime_config);

    return 0;
}

void Yolo8BodyPoseDetector::_update_init_config(std::shared_ptr<InitConfig_t> init_config)
{
    if (m_status != NodeStatusCode::BEFORE_INIT) {
        RDX_RAISE_ERROR("Node status must be BEFORE_INIT, but got {}", m_status);
    }

    m_init_config = init_config;

    // create input port
    m_input_port = std::make_shared<ActionInputPort_t>(this);
    m_input_port->init(init_config->input_port_config);

    // resource booking callback
    m_input_port->set_on_goal_received_callback(
        [this](const auto &goal_uuid, const auto &) {
            // try to book a resource for the goal
            InferenceResource_t inference_resource;
            bool booked = m_impl->inference_resources.try_pop(inference_resource);
            if (booked) {
                // book the resource, and accept the goal
                m_impl->goal_to_inference_resource->insert({goal_uuid, inference_resource});
                return 0;
            }
            // no resource available, reject the goal
            return -1;
        });

    // bind the resource to the source data on accept
    m_input_port->set_on_goal_enqueued_callback(
        [this](std::shared_ptr<SourceData_t> source_data) {
            // lock the goal to inference resource
            auto lock = m_impl->goal_to_inference_resource.synchronize();

            // get the booked resource
            auto inference_resource = lock->find(source_data->get_goal_uuid());

            // should not happen, the resource should be booked before enqueued
            if (inference_resource == lock->end()) {
                RDX_RAISE_ERROR("No booked resource found for goal uuid: {}", to_boost_uuid_string(source_data->get_goal_uuid()));
            }

            // bind the resource to the source data
            source_data->model_resource = inference_resource->second;

            // release the book keeping entry
            lock->erase(source_data->get_goal_uuid());

            return 0;
        });

    // remove booked resource if goal is rejected or canceled
    m_input_port->set_on_goal_rejected_callback(
        [this](const auto &goal_uuid, auto) {
            auto lock = m_impl->goal_to_inference_resource.synchronize();
            auto it = lock->find(goal_uuid);
            if (it != lock->end()) {
                // return resource to pool
                m_impl->inference_resources.push(it->second);
                // remove from book keeping
                lock->erase(it);
            }
            return 0;
        });

    m_input_port->set_on_goal_cancel_request_callback(
        [this](auto goal_handle) {
            auto lock = m_impl->goal_to_inference_resource.synchronize();
            auto it = lock->find(goal_handle->get_goal_id());
            if (it != lock->end()) {
                // return resource to pool
                m_impl->inference_resources.push(it->second);
                // remove from book keeping
                lock->erase(it);
            }
            return 0;
        });

    // create all inference resources
    for (const auto &model_config : init_config->model_configs) {
        auto ret = _create_inference_resource(model_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("Failed to create inference resource, error code: {}", ret);
        }
    }

    // done, state change to STOPPED
    m_status = NodeStatusCode::STOPPED;
}

void Yolo8BodyPoseDetector::_update_runtime_config(std::shared_ptr<RuntimeConfig_t> runtime_config)
{
    if (m_status != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("Node status must be STOPPED, but got {}", m_status);
    }

    // state change to STARTED, then create step thread
    m_runtime_config = runtime_config;
    m_status = NodeStatusCode::STARTED;

    // create step thread, and run it
    m_step_thread = std::make_shared<std::thread>([this]() {
        while (m_status == NodeStatusCode::STARTED && rclcpp::ok()) {
            auto start_time = std::chrono::steady_clock::now();
            _step();
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < m_runtime_config->step_interval) {
                std::this_thread::sleep_for(m_runtime_config->step_interval - elapsed);
            }
        }
    });
}

void Yolo8BodyPoseDetector::_step()
{
    // do nothing if not started
    if (m_status != NodeStatusCode::STARTED) {
        return;
    }

    // get a message from input port
    std::shared_ptr<ActionInputPort_t::SourceData_t> source_data;
    auto use_blocking_mode = m_runtime_config->enable_blocking_mode;
    if (use_blocking_mode) {
        source_data = m_input_port->pop_source_data();
    } else {
        source_data = m_input_port->try_pop_source_data();
    }
    if (!source_data) {
        // no data, do nothing
        return;
    }

    // get image
    auto image = _extract_image(source_data);

    // find a resource and then run inference
    m_impl->inference_task_group.run([this, source_data, image]() {
        auto model = source_data->model_resource->model;
        auto inout_data = source_data->model_resource->inout_data;
        // TODO: run inference

        // return the resource
        m_impl->inference_resources.push(*source_data->model_resource);
    });
}

int Yolo8BodyPoseDetector::_create_inference_resource(
    InitConfig_t::ModelConfig_t::Ptr model_config,
    int replicas)
{
    // create a new inference resource, and push it to the concurrent queue
    // @return 0 if success, -1 if failed
    // load model
    auto model = std::make_shared<inference::yolo8::Yolo8PoseModel>();
    auto model_init_params = model->create_init_params();
    {
        auto _init_params = std::dynamic_pointer_cast<inference::yolo8::Yolo8ModelConfig>(model_init_params);
        if (!_init_params) {
            RDX_RAISE_ERROR("Failed to cast model init params to Yolo8ModelConfig");
        }
        *_init_params = *model_config;
    }
    auto ret_model_open = model->open(model_init_params);
    if (ret_model_open != 0) {
        RDX_RAISE_ERROR("Failed to open model, error code: {}", ret_model_open);
        return -1;
    }

    for (int i = 0; i < replicas; ++i) {
        // create inference inout data
        auto inference_inout_data = model->create_inference_inout_data();

        // create inference resource
        InferenceResource_t inference_resource;
        inference_resource.model = model;
        inference_resource.inout_data = inference_inout_data;
        inference_resource.model_config = model_config;
        inference_resource.replica_id = i;
        m_impl->inference_resources.push(inference_resource);
    }
    return 0;
}

int Yolo8BodyPoseDetector::_create_all_inference_resources(
    const std::vector<InitConfig_t::ModelConfig_t::Ptr> &model_configs)
{
    std::map<InitConfig_t::ModelConfig_t::Ptr, int> model_config_to_replicas;

    // count the number of replicas for each model config
    for (const auto &model_config : model_configs) {
        model_config_to_replicas[model_config] += 1;
    }

    // create inference resources for each model config
    for (const auto &[model_config, replicas] : model_config_to_replicas) {
        auto ret = _create_inference_resource(model_config, replicas);
        if (ret != 0) {
            RDX_RAISE_ERROR("Failed to create inference resource, error code: {}", ret);
            return ret;
        }
    }
    return 0;
}

} // namespace redoxi_works::model_nodes
