#pragma once

#include <redoxi_common_nodes/redoxi_common_nodes.hpp>
#include <redoxi_common_nodes/base_nodes/BaseRosNode.hpp>
#include <rclcpp/rclcpp.hpp>

namespace redoxi_works::common_nodes
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
} // namespace redoxi_works::common_nodes
