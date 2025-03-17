#pragma once

#include <future>
#include <tbb/task_group.h>
#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_cpp/common_concepts.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works::common_nodes::v1
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
class BaseRosNode : public rclcpp::Node
{
  public:
    using InitConfig_t = BaseRosNodeInitConfig;
    using RuntimeConfig_t = BaseRosNodeRuntimeConfig;

    // do not change these names in subclass, otherwise you cannot find out the type of the bottom level config
    using RootInitConfig_t = BaseRosNodeInitConfig;
    using RootRuntimeConfig_t = BaseRosNodeRuntimeConfig;

  public:
    BaseRosNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~BaseRosNode() noexcept;

  public:
    /**
     * @brief Initialize the node with automatically loaded config (using _load_init_config() and _load_runtime_config())
     * @details Must be called once in UNCONFIGURED state
     * @return 0 if success, otherwise error code
     * @note This function is intended to be called only once
     */
    int init();

    /**
     * @brief Initialize the node with external config
     * @details Must be called once in UNCONFIGURED state
     * @param init_config Initial configuration
     * @param runtime_config Runtime configuration
     * @return 0 if success, otherwise error code
     * @note This function is intended to be called only once
     */
    int init(std::shared_ptr<RootInitConfig_t> init_config,
             std::shared_ptr<RootRuntimeConfig_t> runtime_config);

    //! Get json parameters parsed from ros parameters
    const nlohmann::json &get_json_parameters() const
    {
        return m_json_parameters;
    }

    int get_status() const
    {
        return m_status;
    }

    void set_status(int status)
    {
        _on_status_set_before(status);
        m_status = status;
        _on_status_set_after(status);
    }

    //! Get init config
    std::shared_ptr<const RootInitConfig_t> get_init_config() const
    {
        return m_init_config;
    }

    //! Get runtime config
    std::shared_ptr<const RootRuntimeConfig_t> get_runtime_config() const
    {
        return m_runtime_config;
    }

    //! Get init config
    std::shared_ptr<RootInitConfig_t> get_init_config()
    {
        return m_init_config;
    }

    //! Get runtime config
    std::shared_ptr<RootRuntimeConfig_t> get_runtime_config()
    {
        return m_runtime_config;
    }

  protected:
    //! Create init config, intended to be overridden by subclass
    virtual std::shared_ptr<RootInitConfig_t> _load_init_config() const
    {
        // to be compatible with v2, will be called by init() without given configs
        throw std::runtime_error("Not implemented");
    }

    //! Create runtime config, intended to be overridden by subclass
    virtual std::shared_ptr<RootRuntimeConfig_t> _load_runtime_config() const
    {
        // to be compatible with v2, will be called by init() without given configs
        throw std::runtime_error("Not implemented");
    }

    //! Step function to be called periodically, intended to be overridden by subclass
    virtual void _step() = 0;

    //! Callback before status is set
    virtual void _on_status_set_before(int new_status)
    {
        (void)new_status;
    }

    //! Callback after status is set
    virtual void _on_status_set_after(int new_status)
    {
        (void)new_status;
    }

    //! Update init config, intended to be overridden by subclass. After this, m_init_config will be updated.
    //! @return 0 if success, otherwise error code
    virtual int _update_init_config(std::shared_ptr<RootInitConfig_t> init_config) = 0;

    //! Update runtime config, intended to be overridden by subclass. After this, m_runtime_config will be updated.
    //! @return 0 if success, otherwise error code.
    virtual int _update_runtime_config(std::shared_ptr<RootRuntimeConfig_t> runtime_config) = 0;

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

    //! Node status
    std::atomic<int> m_status{NodeStatusCode::BEFORE_INIT};

    //! Thread for periodic step
    std::shared_ptr<std::thread> m_step_thread;
    std::atomic<bool> m_step_running{false};

    //! Init config
    std::shared_ptr<RootInitConfig_t> m_init_config;

    //! Runtime config
    std::shared_ptr<RootRuntimeConfig_t> m_runtime_config;

    //! Task group for executing async tasks not in the calling thread
    tbb::task_group m_async_task_group;

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

} // namespace redoxi_works::common_nodes::v1
