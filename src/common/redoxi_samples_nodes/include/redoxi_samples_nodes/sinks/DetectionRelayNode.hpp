#pragma once

#include <redoxi_common_nodes/detection_ports/DetectionResponseInputPort.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessReplyHandler.hpp>
#include <json_struct/json_struct.h>

namespace redoxi_works
{

struct DetectionRelayNodeInitConfig : public common_nodes::StartStopNode::InitConfig_t {
    using InputPort_t = detection_ports::response_only::DetectionResponseInputPort;
    std::shared_ptr<InputPort_t::InitConfig_t> input_port_config = std::make_shared<InputPort_t::InitConfig_t>();

    //! the topic to publish the relayed detection
    std::string publish_detection_topic = "out/relayed_detection";
    std::string publish_visualization_topic = "out/relayed_visualization";

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
                         JS_MEMBER(input_port_config),
                         JS_MEMBER(publish_detection_topic),
                         JS_MEMBER(publish_visualization_topic));
};

struct DetectionRelayNodeRuntimeConfig : public common_nodes::StartStopNode::RuntimeConfig_t {
    bool enable_blocking_mode = false;
    bool enable_visualization = true;

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::RuntimeConfig_t),
                         JS_MEMBER(enable_blocking_mode),
                         JS_MEMBER(enable_visualization));
};

class DetectionRelayNode : public common_nodes::StartStopNode
{
  public:
    DetectionRelayNode(const std::string &node_name,
                       const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  public: // useful types
    using InputPort_t = detection_ports::response_only::DetectionResponseInputPort;
    using SourceData_t = InputPort_t::SourceData_t;
    using ActionDataTrait_t = InputPort_t::ActionDataTrait_t;
    using InitConfig_t = DetectionRelayNodeInitConfig;
    using RuntimeConfig_t = DetectionRelayNodeRuntimeConfig;

    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;

  protected:
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;

  protected:
    int _parse_detection(cv::Mat *output, const SourceData_t &source_data);
    int _create_port_handler(std::shared_ptr<RuntimeConfig_t> runtime_config);

  protected:
    using PortHandler_t = port_handlers::PullProcessReplyHandler<InputPort_t::MasterSpec_t>;
    std::shared_ptr<InputPort_t> m_input_port;
    StampedImagePub m_pub_visualization;
    std::shared_ptr<PortHandler_t> m_port_handler;
};
} // namespace redoxi_works