#pragma once

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_nodes/base_nodes/v2/BaseRosNode.hpp>
#include <rclcpp/rclcpp.hpp>

namespace redoxi_works::common_nodes::v2
{

//! Base class for node that supports start/stop
class StartStopNode : public BaseRosNode,
                      public IStartStopProtocol
{
  public:
    StartStopNode(const std::string &node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  public:
    int start() final;
    int stop() final;

  public:
    // ros lifecycle callbacks

    // init() if necessary, UNCONFIGURED -> INACTIVE
    RosLifecycleCallbackReturn_t on_configure(const RosLifecycleState_t &state) final;

    // _start(), INACTIVE -> ACTIVE
    RosLifecycleCallbackReturn_t on_activate(const RosLifecycleState_t &state) final;

    // _stop(), ACTIVE -> INACTIVE
    RosLifecycleCallbackReturn_t on_deactivate(const RosLifecycleState_t &state) final;

  protected:
    //! async stop the node, after which you can start it again
    //! @note This function is intended to be called in the step thread
    //! @return the future which will be resolved into what stop() returns
    std::shared_future<int> _async_stop();

    //! async start the node, after which you can stop it again
    //! @note This function is intended to be called in the step thread
    //! @return the future which will be resolved into what start() returns
    std::shared_future<int> _async_start();

    //! callback when the stopping is finished
    //! @return 0 if success, otherwise error code
    virtual int _on_stopped()
    {
        return 0;
    };

    //! callback when the starting is finished
    //! @return 0 if success, otherwise error code
    virtual int _on_started()
    {
        return 0;
    };

  protected:
    //! subclass should implement this function, with state transition handled by base class
    //! @return 0 if success, otherwise error code
    virtual int _start() = 0;

    //! subclass should implement this function, with state transition handled by base class
    //! @return 0 if success, otherwise error code
    virtual int _stop() = 0;

    //! subclass should implement this function, with state transition handled by base class
    //! @return Node status after init.
    virtual int _get_state_after_init() const override
    {
        return NodeStatusCode::STOPPED;
    }
};
} // namespace redoxi_works::common_nodes::v2
