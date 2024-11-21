#include <chrono>
#include <thread>
#include <redoxi_common_nodes/base_nodes/BaseRosNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::common_nodes
{

// void BaseRosNodeInitConfig::from_parameters(const BaseRosNode *node)
// {
//     parse_from_node_parameters(this, node);
//     // RDX_INFO_DEV(node, __func__, false, "{}", "load init config from node");
//     // auto &json_params = node->get_json_parameters();

//     // //! Load init config from json parameters if exists
//     // if (json_params.contains("init_config")) {
//     //     RDX_INFO_DEV(node, __func__, false, "{}", "found init_config in json parameters");

//     //     std::string json_str = json_params["init_config"].dump();
//     //     JS::ParseContext context(json_str);
//     //     auto error = context.parseTo(*this);
//     //     if (error != JS::Error::NoError) {
//     //         RDX_RAISE_ERROR("[{}] Error parsing init_config: {}", __func__, context.makeErrorString());
//     //     }
//     // }

//     // RDX_INFO_DEV(node, __func__, false, "{}", "init config loaded");
// }

// void BaseRosNodeRuntimeConfig::from_parameters(const BaseRosNode *node)
// {
//     parse_from_node_parameters(this, node);
//     // RDX_INFO_DEV(node, __func__, false, "{}", "load runtime config from node");
//     // auto &json_params = node->get_json_parameters();

//     // //! Load runtime config from json parameters if exists
//     // if (json_params.contains("runtime_config")) {
//     //     RDX_INFO_DEV(node, __func__, false, "{}", "found runtime_config in json parameters");
//     //     std::string json_str = json_params["runtime_config"].dump();
//     //     JS::ParseContext context(json_str);
//     //     auto error = context.parseTo(*this);
//     //     if (error != JS::Error::NoError) {
//     //         RDX_RAISE_ERROR("[{}] Error parsing runtime_config: {}", __func__, context.makeErrorString());
//     //     }
//     // }

//     // RDX_INFO_DEV(node, __func__, false, "{}", "runtime config loaded");
// }

BaseRosNode::BaseRosNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options)
{
    auto ret = declare_default_parameters_for_node(this);
    if (ret != 0) {
        RDX_RAISE_ERROR("Failed to declare default parameters for node {}", node_name);
    }

    // get json parameters
    auto node = this;
    m_json_parameters = RDX_GET_JSON_PARAM_FROM_NODE(node);
}

int BaseRosNode::set_runtime_config(std::shared_ptr<BaseRosNodeRuntimeConfig> runtime_config)
{
    //! Check state is CLOSED or STOPPED
    if (m_status != NodeStatusCode::CLOSED && m_status != NodeStatusCode::STOPPED) {
        RDX_RAISE_ERROR("[f={}] Node cannot be updated in state {}", __func__, m_status);
    }

    // update runtime config
    {
        auto ret = _update_runtime_config(runtime_config);
        if (ret != 0) {
            RDX_INFO_DEV(this, __func__, false, "{}", "failed to update runtime config");
            return ret;
        }
        m_runtime_config = runtime_config;
    }

    return 0;
}

int BaseRosNode::init(std::shared_ptr<BaseRosNodeInitConfig> init_config,
                      std::shared_ptr<BaseRosNodeRuntimeConfig> runtime_config)
{
    //! Check state is BEFORE_INIT
    if (m_status != NodeStatusCode::BEFORE_INIT) {
        RDX_RAISE_ERROR("[f={}] Node cannot be initialized in state {}", __func__, m_status);
    }

    // update init config
    {
        auto ret = _update_init_config(init_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to update init config", __func__);
        }
        m_init_config = init_config;
    }

    // update runtime config
    {
        auto ret = _update_runtime_config(runtime_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("[f={}] Failed to update runtime config", __func__);
        }
        m_runtime_config = runtime_config;
    }

    set_status(_get_state_after_init());

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

BaseRosNode::~BaseRosNode()
{
    _internal_stop_step_thread();
}

} // namespace redoxi_works::common_nodes
