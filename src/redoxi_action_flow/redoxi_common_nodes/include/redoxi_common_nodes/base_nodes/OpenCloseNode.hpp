#pragma once

#include <redoxi_common_nodes/base_nodes/BaseRosNode.hpp>

namespace redoxi_works::common_nodes
{

//! Base class for node that supports open/close/start/stop
//! @note: this is a stateful node, the status code is used to indicate the current state
//! state is managed by lifecycle node
//! UNCONFIGURED -> [init() once] -> open() -> INACTIVE -> start() -> ACTIVE -> stop() -> INACTIVE -> close() -> UNCONFIGURED -> shutdown() -> FINALIZED
class OpenCloseNode : public BaseRosNode,
                      public IOpenCloseProtocol
{
  public:
    using BaseRosNode::BaseRosNode;
    virtual ~OpenCloseNode() noexcept = default;

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
    RosLifecycleCallbackReturn_t on_configure(const RosLifecycleState_t &state) final;

    // _start(), INACTIVE -> ACTIVE
    RosLifecycleCallbackReturn_t on_activate(const RosLifecycleState_t &state) final;

    // _stop(), ACTIVE -> INACTIVE
    RosLifecycleCallbackReturn_t on_deactivate(const RosLifecycleState_t &state) final;

    // _close(), INACTIVE -> UNCONFIGURED
    RosLifecycleCallbackReturn_t on_cleanup(const RosLifecycleState_t &state) final;

    // _shutdown(), ANYTHING -> FINALIZED
    RosLifecycleCallbackReturn_t on_shutdown(const RosLifecycleState_t &state) final;

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
    //! If called in opened state, it should do nothing and return 0
    //! @return 0 if success, otherwise error code
    virtual int _open() = 0;

    //! subclass should implement this function, with state transition handled by base class
    //! If called in closed state, it should do nothing and return 0
    //! @return 0 if success, otherwise error code
    virtual int _close() = 0;

    //! subclass should implement this function, with state transition handled by base class
    //! If called in closed state, it should do nothing and return 0
    //! @return 0 if success, otherwise error code
    virtual int _start() = 0;

    //! subclass should implement this function, with state transition handled by base class
    //! If called in started state, it should do nothing and return 0
    //! @return 0 if success, otherwise error code
    virtual int _stop() = 0;

    //! subclass should implement this function, with state transition handled by base class
    //! @return Node status after init.
    virtual int _get_state_after_init() const override
    {
        return NodeStatusCode::CLOSED;
    }
};
} // namespace redoxi_works::common_nodes