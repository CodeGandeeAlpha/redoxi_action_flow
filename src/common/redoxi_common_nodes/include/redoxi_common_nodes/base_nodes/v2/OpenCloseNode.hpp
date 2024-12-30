#pragma once

#include <redoxi_common_nodes/base_nodes/v2/BaseRosNode.hpp>

namespace redoxi_works::common_nodes::v2
{

//! Base class for node that supports open/close/start/stop
//! @note: this is a stateful node, the status code is used to indicate the current state
//! state is managed by lifecycle node
//! UNCONFIGURED -> [init() once] -> open() -> INACTIVE -> start() -> ACTIVE -> stop() -> INACTIVE -> close() -> UNCONFIGURED -> shutdown() -> FINALIZED
class OpenCloseNode : public BaseRosNode,
                      public IOpenCloseProtocol
{
  public:
    OpenCloseNode(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    virtual ~OpenCloseNode() noexcept
    {
    }

  public:
    //! open the node, if you want to update init_config, do it before calling this function
    int open() final;

    //! close the node, after which you can open it again
    int close() final;

    //! start the node, if you want to update runtime_config, do it before calling this function
    int start() final;

    //! stop the node, after which you can start it again
    int stop() final;

  public:
    // ros lifecycle callbacks

    // init() if necessary, and then _open(), UNCONFIGURED -> INACTIVE
    RosLifecycleCallbackReturn_t on_configure(const RosLifecycleState_t &state) override;

    // _start(), INACTIVE -> ACTIVE
    RosLifecycleCallbackReturn_t on_activate(const RosLifecycleState_t &state) override;

    // _stop(), ACTIVE -> INACTIVE
    RosLifecycleCallbackReturn_t on_deactivate(const RosLifecycleState_t &state) override;

    // _close(), INACTIVE -> UNCONFIGURED
    RosLifecycleCallbackReturn_t on_cleanup(const RosLifecycleState_t &state) override;

  protected:
    //! async close the node, after which you can open it again
    //! @note This function is intended to be called in the step thread
    //! @return the future which will be resolved into what close() returns
    std::shared_future<int> _async_close();

    //! async stop the node, after which you can start it again
    //! @note This function is intended to be called in the step thread
    //! @return the future which will be resolved into what stop() returns
    std::shared_future<int> _async_stop();

    //! async open the node, after which you can start it
    //! @note This function is intended to be called in the step thread
    //! @return the future which will be resolved into what open() returns
    std::shared_future<int> _async_open();

    //! async start the node, after which you can stop it
    //! @note This function is intended to be called in the step thread
    //! @return the future which will be resolved into what start() returns
    std::shared_future<int> _async_start();

    //! callback when the closing is finished
    //! @return 0 if success, otherwise error code
    virtual int _on_closed()
    {
        return 0;
    };

    //! callback when the opening is finished
    //! @return 0 if success, otherwise error code
    virtual int _on_opened()
    {
        return 0;
    };

    //! callback when the starting is finished
    //! @return 0 if success, otherwise error code
    virtual int _on_started()
    {
        return 0;
    };

    //! callback when the stopping is finished
    //! @return 0 if success, otherwise error code
    virtual int _on_stopped()
    {
        return 0;
    };

  protected:
    //! subclass should implement this function, with state transition handled by base class
    //! @return 0 if success, otherwise error code
    virtual int _open() = 0;

    //! subclass should implement this function, with state transition handled by base class
    //! @return 0 if success, otherwise error code
    virtual int _close() = 0;

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
        return NodeStatusCode::CLOSED;
    }
};
} // namespace redoxi_works::common_nodes::v2