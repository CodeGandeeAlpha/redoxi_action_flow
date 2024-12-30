#pragma once

#include <future>
#include <tbb/task_group.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <json_struct/json_struct.h>

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_cpp/common_concepts.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::common_nodes::v2
{

class BaseRosNode;

// template <TimeDurationConcept TimeUnit = DefaultTimeUnit_t>
class BaseRosNodeInitConfig
{
  public:
    using TimeUnit_t = DefaultTimeUnit_t;

    //! for annotation only, do not change it
    std::string _time_unit = _get_time_unit_name<TimeUnit_t>();

  public:
    virtual ~BaseRosNodeInitConfig() = default;

    //! Parse node parameters to fill this config, this is required because json_struct
    //! does not work with virtual functions
    template <typename T>
    requires std::is_base_of_v<BaseRosNodeInitConfig, T>
    static void parse_from_node_parameters(T *config, const BaseRosNode *node);

  public:
    JS_OBJECT(JS_MEMBER(_time_unit));
};

// template <TimeDurationConcept TimeUnit = DefaultTimeUnit_t>
class BaseRosNodeRuntimeConfig
{
  public:
    using TimeUnit_t = DefaultTimeUnit_t;
    inline static constexpr TimeUnit_t DefaultStepInterval = TimeUnit_t{std::chrono::milliseconds(2)};

  public:
    virtual ~BaseRosNodeRuntimeConfig() = default;

    //! Parse node parameters to fill this config, this is required because json_struct
    //! does not work with virtual functions
    template <typename T>
    requires std::is_base_of_v<BaseRosNodeRuntimeConfig, T>
    static void parse_from_node_parameters(T *config, const BaseRosNode *node);

  public:
    //! for annotation only, do not change it
    std::string _time_unit = _get_time_unit_name<TimeUnit_t>();

    //! Step interval
    TimeUnit_t step_interval = DefaultStepInterval;

  public:
    JS_OBJECT(JS_MEMBER(_time_unit), JS_MEMBER(step_interval));
};

/**
 * @brief Base class for all ROS nodes, with step function
 */
class BaseRosNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
    using InitConfig_t = BaseRosNodeInitConfig;
    using RuntimeConfig_t = BaseRosNodeRuntimeConfig;
    using RosBaseNode_t = rclcpp_lifecycle::LifecycleNode;
    using RosPrimaryState_t = lifecycle_msgs::msg::State;
    using RosLifecycleState_t = rclcpp_lifecycle::State;
    using RosLifecycleTransition_t = rclcpp_lifecycle::Transition;
    using RosLifecycleCallbackReturn_t = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  public:
    BaseRosNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~BaseRosNode() noexcept;

  public:
    //! Initialize the node, must be called once in UNCONFIGURED state
    //! @return 0 if success, otherwise error code.
    //! @note This function is intended to be called only once.
    int init();

    //! Check if init() is called once
    bool check_is_already_init() const
    {
        return m_is_already_init;
    }

    //! Update runtime config, only applicable when the node is CLOSED or STOPPED
    //! @return 0 if success, otherwise error code.
    int set_runtime_config(std::shared_ptr<BaseRosNodeRuntimeConfig> runtime_config);

    //! Get json parameters parsed from ros parameters
    const nlohmann::json &get_json_parameters() const
    {
        return m_json_parameters;
    }

    //! @deprecated Use get_current_state() instead
    [[deprecated("Use get_current_state() instead")]] int get_status() const
    {
        return get_current_state().id();
    }

    //! Get init config
    std::shared_ptr<BaseRosNodeInitConfig> get_init_config() const
    {
        return m_init_config;
    }

    //! Get runtime config
    std::shared_ptr<BaseRosNodeRuntimeConfig> get_runtime_config() const
    {
        return m_runtime_config;
    }

  protected:
    //! Initialize the node, will be called once in UNCONFIGURED state,
    //! intended to be overridden by subclass, configs are created by _load_init_config() and _load_runtime_config()
    virtual int _init(std::shared_ptr<BaseRosNodeInitConfig> init_config,
                      std::shared_ptr<BaseRosNodeRuntimeConfig> runtime_config) = 0;

    //! Create init config, intended to be overridden by subclass
    virtual std::shared_ptr<BaseRosNodeInitConfig> _load_init_config() const = 0;

    //! Create runtime config, intended to be overridden by subclass
    virtual std::shared_ptr<BaseRosNodeRuntimeConfig> _load_runtime_config() const = 0;

    //! Step function to be called periodically, intended to be overridden by subclass
    virtual void _step() = 0;

    //! Update init config, intended to be overridden by subclass. After this, m_init_config will be updated.
    //! @return 0 if success, otherwise error code
    virtual int _update_init_config(std::shared_ptr<BaseRosNodeInitConfig> init_config) = 0;

    //! Update runtime config, intended to be overridden by subclass. After this, m_runtime_config will be updated.
    //! @return 0 if success, otherwise error code.
    virtual int _update_runtime_config(std::shared_ptr<BaseRosNodeRuntimeConfig> runtime_config) = 0;

    //! State transition after init, intended to be overridden by subclass.
    //! @return Node status after init.
    virtual int _get_state_after_init() const = 0;

  protected:
    //! Start the step thread, note that you cannot call this function in the step thread
    virtual void _start_step_thread();

    //! Stop the step thread, note that you cannot call this function in the step thread
    virtual void _stop_step_thread();

    //! Stop the step thread asynchronously, call be called in the step thread
    std::shared_future<void> _async_stop_step_thread();

  protected:
    //! Json parameters read from ros parameters
    nlohmann::json m_json_parameters;

    //! Thread for periodic step
    std::shared_ptr<std::thread> m_step_thread;
    std::atomic<bool> m_step_running{false};

    //! Init config
    std::shared_ptr<BaseRosNodeInitConfig> m_init_config;

    //! Runtime config
    std::shared_ptr<BaseRosNodeRuntimeConfig> m_runtime_config;

    //! Task group for executing async tasks not in the calling thread
    tbb::task_group m_async_task_group;

    //! init() called once?
    std::atomic<bool> m_is_already_init{false};

  private:
    //! make it private so that we can use it in constructor and destructor
    void _internal_stop_step_thread();
    void _internal_start_step_thread();
};

//! Load config from node parameters
template <typename ConfigType>
requires std::is_base_of_v<BaseRosNodeRuntimeConfig, ConfigType>
void BaseRosNodeRuntimeConfig::parse_from_node_parameters(ConfigType *config, const BaseRosNode *node)
{
    RDX_INFO_DEV(node, __func__, false, "{}", "load runtime config from node");
    auto &json_params = node->get_json_parameters();

    //! Load config from json parameters if exists
    if (json_params.contains("runtime_config")) {
        RDX_INFO_DEV(node, __func__, false, "{}", "found runtime_config in json parameters");
        std::string json_str = json_params["runtime_config"].dump();
        JS::ParseContext context(json_str);
        auto error = context.parseTo(*config);
        if (error != JS::Error::NoError) {
            RDX_RAISE_ERROR("[{}] Error parsing config: {}", __func__, context.makeErrorString());
        }
    }

    RDX_INFO_DEV(node, __func__, false, "{}", "runtime config loaded");
}

template <typename ConfigType>
requires std::is_base_of_v<BaseRosNodeInitConfig, ConfigType>
void BaseRosNodeInitConfig::parse_from_node_parameters(ConfigType *config, const BaseRosNode *node)
{
    RDX_INFO_DEV(node, __func__, false, "{}", "load init config from node");
    auto &json_params = node->get_json_parameters();

    //! Load init config from json parameters if exists
    if (json_params.contains("init_config")) {
        RDX_INFO_DEV(node, __func__, false, "{}", "found init_config in json parameters");

        std::string json_str = json_params["init_config"].dump();
        JS::ParseContext context(json_str);
        auto error = context.parseTo(*config);
        if (error != JS::Error::NoError) {
            RDX_RAISE_ERROR("[{}] Error parsing init_config: {}", __func__, context.makeErrorString());
        }
    }

    RDX_INFO_DEV(node, __func__, false, "{}", "init config loaded");
}

} // namespace redoxi_works::common_nodes::v2
