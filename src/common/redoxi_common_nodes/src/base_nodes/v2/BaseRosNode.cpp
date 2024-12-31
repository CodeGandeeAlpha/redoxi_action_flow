#include <redoxi_common_nodes/_pch.hpp>

#include <chrono>
#include <thread>
#include <redoxi_common_nodes/base_nodes/v2/BaseRosNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::common_nodes::v2
{

BaseRosNode::BaseRosNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode(node_name, options)
{
    auto ret = declare_default_parameters_for_node(this);
    if (ret != 0) {
        RDX_RAISE_ERROR("Failed to declare default parameters for node {}", node_name);
    }

    // get json parameters
    auto node = this;
    m_json_parameters = RDX_GET_JSON_PARAM_FROM_LIFECYCLE_NODE(node);
}

BaseRosNode::BaseRosNode(const rclcpp::NodeOptions &options)
    : BaseRosNode("BaseRosNode", options)
{
}

std::shared_future<void> BaseRosNode::_async_stop_step_thread()
{
    // make sure the step thread cannot proceed anymore
    m_step_running = false;

    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    m_async_task_group.run([promise, this]() {
        _internal_stop_step_thread();
        promise->set_value();
    });

    return future;
}

int BaseRosNode::init()
{
    auto init_config = _load_init_config();
    auto runtime_config = _load_runtime_config();

    return init(init_config, runtime_config);
}

int BaseRosNode::init(std::shared_ptr<RootInitConfig_t> init_config,
                      std::shared_ptr<RootRuntimeConfig_t> runtime_config)
{
    // must be in configuring state or unconfigured state
    {
        auto state = get_current_state();
        if (state.id() != lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING &&
            state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            RDX_RAISE_ERROR("[f={}()] Node cannot be initialized in state {}", __func__, state.id());
        }
    }

    // init() can only be called once
    if (m_is_already_init) {
        RDX_RAISE_ERROR("[f={}()] Node cannot be initialized more than once", __func__);
        return -1;
    }

    {
        auto ret = _update_init_config(init_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to update init config", __func__);
        }
        m_init_config = init_config;
    }

    {
        auto ret = _update_runtime_config(runtime_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to update runtime config", __func__);
        }
        m_runtime_config = runtime_config;
    }

    // flag as initialized
    m_is_already_init = true;
    return 0;
}

void BaseRosNode::_start_step_thread()
{
    _internal_start_step_thread();
}

void BaseRosNode::_internal_start_step_thread()
{
    m_step_running = true;
    auto interval = m_runtime_config->step_interval;

    m_step_thread = std::make_shared<std::thread>([this, interval]() {
        while (rclcpp::ok() && m_step_running) {
            // sleep no more than interval, taking into account the time taken by _step()
            auto start = std::chrono::steady_clock::now();
            _step();
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed < interval) {
                std::this_thread::sleep_for(interval - elapsed);
            }
        }
    });
}

void BaseRosNode::_stop_step_thread()
{
    _internal_stop_step_thread();
}

void BaseRosNode::_internal_stop_step_thread()
{
    //! Stop step thread
    m_step_running = false;
    if (m_step_thread && m_step_thread->joinable()) {
        m_step_thread->join();
    }
}

BaseRosNode::~BaseRosNode() noexcept
{
    _internal_stop_step_thread();

    // wait for all async tasks to finish
    m_async_task_group.wait();
}

} // namespace redoxi_works::common_nodes::v2
