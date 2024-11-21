#pragma once

#include <redoxi_common_nodes/base_nodes/BaseRosNode.hpp>

namespace redoxi_works::common_nodes
{

//! Base class for node that supports open/close/start/stop
class OpenCloseNode : public BaseRosNode,
                      public IOpenCloseProtocol
{
  public:
    OpenCloseNode(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  public:
    int open() final;
    int close() final;
    int start() final;
    int stop() final;

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
} // namespace redoxi_works::common_nodes