#pragma once
#include <universal_mot_trackers/visibility_control.h>
#include <universal_mot_trackers/UniversalMotTrackerTypes.hpp>
#include <redoxi_common_nodes/base_nodes/OpenCloseNode.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessReplyHandler.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <std_msgs/msg/string.hpp>

namespace redoxi_works::model_nodes::universal_mot_trackers
{
class UniversalMotTrackerNode : public common_nodes::OpenCloseNode
{
  public:
    UniversalMotTrackerNode(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    using BaseNode_t = common_nodes::OpenCloseNode;
    using BaseInitConfig_t = BaseNode_t::InitConfig_t;
    using BaseRuntimeConfig_t = BaseNode_t::RuntimeConfig_t;

    using InitConfig_t = types::InitConfig;
    using RuntimeConfig_t = types::RuntimeConfig;
    using InputPort_t = types::TrackingRequestInputPort;
    using InputAction_t = typename InputPort_t::ActionType_t;
    using InputActionResult_t = typename InputPort_t::ActionResult_t;
    using InputActionGoal_t = typename InputPort_t::ActionGoal_t;
    using InputActionDataTrait_t = typename InputPort_t::ActionDataTrait_t;
    using InputSourceData_t = typename InputPort_t::SourceData_t;
    using InputPortHandler_t = port_handlers::PullProcessReplyHandler<typename InputPort_t::MasterSpec_t>;

  public:
    void _step() override;
    int _open() override;
    int _close() override;
    int _start() override;
    int _stop() override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config) override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> config) override;

  protected:
    struct Impl;
    std::shared_ptr<Impl> m_impl;
    std::shared_ptr<InputPort_t> m_input_port;
    std::shared_ptr<StampedImagePub> m_pub_visualization;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_pub_probe;

  private:
    int _create_tracker(const InitConfig_t &init_config);
};
} // namespace redoxi_works::model_nodes::universal_mot_trackers
