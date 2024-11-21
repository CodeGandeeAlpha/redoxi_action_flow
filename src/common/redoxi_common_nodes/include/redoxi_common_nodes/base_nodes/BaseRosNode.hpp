#pragma once

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_cpp/common_concepts.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works::common_nodes
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
    virtual void from_parameters(const BaseRosNode *node);

  public:
    JS_OBJECT(JS_MEMBER(_time_unit));
};

// template <TimeDurationConcept TimeUnit = DefaultTimeUnit_t>
class BaseRosNodeRuntimeConfig
{
  public:
    using TimeUnit_t = DefaultTimeUnit_t;
    inline static constexpr TimeUnit_t DefaultStepInterval = TimeUnit_t{std::chrono::milliseconds(5)};

  public:
    virtual ~BaseRosNodeRuntimeConfig() = default;
    virtual void from_parameters(const BaseRosNode *node);

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

  public:
    BaseRosNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~BaseRosNode();

  public:
    //! Initialize the node, default state transition: BEFORE_INIT -> CLOSED
    //! @return 0 if success, otherwise error code.
    //! @note This function is intended to be called only once.
    int init(std::shared_ptr<BaseRosNodeInitConfig> init_config,
             std::shared_ptr<BaseRosNodeRuntimeConfig> runtime_config);

    //! Update runtime config, only applicable when the node is CLOSED or STOPPED
    //! @return 0 if success, otherwise error code.
    int set_runtime_config(std::shared_ptr<BaseRosNodeRuntimeConfig> runtime_config);

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

  protected:
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
    virtual int _update_init_config(std::shared_ptr<BaseRosNodeInitConfig> init_config) = 0;

    //! Update runtime config, intended to be overridden by subclass. After this, m_runtime_config will be updated.
    //! @return 0 if success, otherwise error code.
    virtual int _update_runtime_config(std::shared_ptr<BaseRosNodeRuntimeConfig> runtime_config) = 0;

    //! State transition after init, intended to be overridden by subclass.
    //! @return Node status after init.
    virtual int _get_state_after_init() const = 0;

  protected:
    //! Start the step thread
    virtual void _start_step_thread();

    //! Stop the step thread
    virtual void _stop_step_thread();

  protected:
    //! Json parameters read from ros parameters
    nlohmann::json m_json_parameters;

    //! Node status
    std::atomic<int> m_status{NodeStatusCode::BEFORE_INIT};

    //! Thread for periodic step
    std::shared_ptr<std::thread> m_step_thread;
    std::atomic<bool> m_step_running{false};

    //! Init config
    std::shared_ptr<BaseRosNodeInitConfig> m_init_config;

    //! Runtime config
    std::shared_ptr<BaseRosNodeRuntimeConfig> m_runtime_config;

  private:
    //! make it private so that we can use it in constructor and destructor
    void _internal_stop_step_thread();
    void _internal_start_step_thread();
};
} // namespace redoxi_works::common_nodes
