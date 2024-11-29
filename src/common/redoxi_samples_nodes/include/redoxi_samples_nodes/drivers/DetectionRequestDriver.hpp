#pragma once

#include <memory>

#include <redoxi_common_nodes/visibility_control.h>
#include <redoxi_common_nodes/image_ports/AsyncImageInputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionRequestOutputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionResponseOutputPort.hpp>
#include <redoxi_common_nodes/base_nodes/StartStopNode.hpp>
#include <redoxi_common_nodes/port_handlers/PullProcessSendHandler.hpp>
#include <redoxi_common_cpp/ros_utils/StampedImagePub.hpp>

namespace redoxi_works::samples_nodes::drivers
{
struct DetectionRequestDriverInitConfig;
struct DetectionRequestDriverRuntimeConfig;

//! Driver for detection request
//! Receives images from image input port, and sends out detection requests
//! Retrieve the result from goal handle, and then send out detection response
class DetectionRequestDriver : public common_nodes::StartStopNode
{
  public:
    struct ByImageRequest {
        using InputPortSpec_t = redoxi_works::image_ports::types::ImageActionInputPortSpec;
        using InputPort_t = redoxi_works::image_ports::AsyncImageInputPort;
        using Action_t = InputPortSpec_t::ActionType_t;
        using ActionDataTrait_t = InputPortSpec_t::ActionDataTrait_t;
        using ActionGoal_t = typename Action_t::Goal;
        using ActionResult_t = typename Action_t::Result;
        using SourceData_t = typename InputPortSpec_t::ReceiveSourceData_t;
    };

    using InitConfig_t = DetectionRequestDriverInitConfig;
    using BaseInitConfig_t = common_nodes::StartStopNode::InitConfig_t;
    using RuntimeConfig_t = DetectionRequestDriverRuntimeConfig;
    using BaseRuntimeConfig_t = common_nodes::StartStopNode::RuntimeConfig_t;

    using DetectionRequestOutputPort_t = detection_ports::request_response::DetectionRequestOutputPort;
    using DetectionRequestOutputPortSpec_t = DetectionRequestOutputPort_t::MasterSpec_t;
    using DetectionRequestOutputSourceData_t = typename DetectionRequestOutputPort_t::SourceData_t;
    using DetectionRequestOutputRequest_t = typename DetectionRequestOutputPort_t::DeliveryRequest_t;
    using DetectionRequestOutputAction_t = DetectionRequestOutputPortSpec_t::ActionType_t;
    using DetectionRequestOutputActionDataTrait_t = DetectionRequestOutputPortSpec_t::ActionDataTrait_t;
    using DetectionRequestOutputActionGoal_t = typename DetectionRequestOutputAction_t::Goal;
    using DetectionRequestOutputActionResult_t = typename DetectionRequestOutputAction_t::Result;

    using DetectionResponseOutputPort_t = detection_ports::response_only::DetectionResponseOutputPort;
    using DetectionResponseOutputPortSpec_t = DetectionResponseOutputPort_t::MasterSpec_t;
    using DetectionResponseOutputSourceData_t = typename DetectionResponseOutputPort_t::SourceData_t;
    using DetectionResponseOutputRequest_t = typename DetectionResponseOutputPort_t::DeliveryRequest_t;
    using DetectionResponseOutputAction_t = DetectionResponseOutputPortSpec_t::ActionType_t;
    using DetectionResponseOutputActionDataTrait_t = DetectionResponseOutputPortSpec_t::ActionDataTrait_t;
    using DetectionResponseOutputActionGoal_t = typename DetectionResponseOutputAction_t::Goal;
    using DetectionResponseOutputActionResult_t = typename DetectionResponseOutputAction_t::Result;

  public:
    DetectionRequestDriver(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  protected:
    using ImageRequestPortHandler_t = port_handlers::PullProcessSendHandler<ByImageRequest::InputPortSpec_t,
                                                                            DetectionRequestOutputPortSpec_t>;
    int _start() override;
    int _stop() override;
    void _step() override;
    int _update_init_config(std::shared_ptr<BaseInitConfig_t> init_config) override;
    int _update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config) override;
    int _extract_image(cv::Mat *output_image, const ByImageRequest::SourceData_t &source_data);

    void _on_detection_request_sent(DetectionRequestOutputPort_t::TargetData_t &target_data,
                                    DetectionRequestOutputPort_t::SendResult_t &send_result,
                                    const DetectionRequestOutputPort_t::DeliveryRequest_t &delivery_request,
                                    const DetectionRequestOutputPort_t::Downstream_t &downstream);

  protected:
    std::shared_ptr<ByImageRequest::InputPort_t> m_image_input_port;
    std::shared_ptr<DetectionRequestOutputPort_t> m_detection_request_output_port;
    std::shared_ptr<DetectionResponseOutputPort_t> m_detection_response_output_port;
    std::shared_ptr<StampedImagePub> m_pub_visualization;
    std::shared_ptr<ImageRequestPortHandler_t> m_image_request_port_handler;
};

struct DetectionRequestDriverInitConfig : public common_nodes::StartStopNode::InitConfig_t {
    using ImageInputPortConfig_t = DetectionRequestDriver::ByImageRequest::InputPort_t::InitConfig_t;
    using DetectionRequestOutputPortConfig_t = DetectionRequestDriver::DetectionRequestOutputPort_t::InitConfig_t;
    using DetectionResponseOutputPortConfig_t = DetectionRequestDriver::DetectionResponseOutputPort_t::InitConfig_t;

    std::shared_ptr<ImageInputPortConfig_t> input_port_config = std::make_shared<ImageInputPortConfig_t>();
    std::shared_ptr<DetectionRequestOutputPortConfig_t> detection_request_output_port_config = std::make_shared<DetectionRequestOutputPortConfig_t>();
    std::shared_ptr<DetectionResponseOutputPortConfig_t> detection_response_output_port_config = std::make_shared<DetectionResponseOutputPortConfig_t>();

    std::string publish_visualization_topic = "out/visualization";

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::InitConfig_t),
                         JS_MEMBER(input_port_config),
                         JS_MEMBER(detection_request_output_port_config),
                         JS_MEMBER(detection_response_output_port_config),
                         JS_MEMBER(publish_visualization_topic));
};

struct DetectionRequestDriverRuntimeConfig : public common_nodes::StartStopNode::RuntimeConfig_t {
    using DetectionRequestOutputPortDeliveryPolicy_t = DetectionRequestDriver::DetectionRequestOutputPort_t::DeliveryPolicy_t;
    using DetectionResponseOutputPortDeliveryPolicy_t = DetectionRequestDriver::DetectionResponseOutputPort_t::DeliveryPolicy_t;
    DetectionRequestOutputPortDeliveryPolicy_t detection_request_enqueue_policy;
    DetectionResponseOutputPortDeliveryPolicy_t detection_response_enqueue_policy;
    bool enable_visualization = true;
    bool enable_blocking_mode = false;

    JS_OBJECT_WITH_SUPER(JS_SUPER(common_nodes::StartStopNode::RuntimeConfig_t),
                         JS_MEMBER(detection_request_enqueue_policy),
                         JS_MEMBER(detection_response_enqueue_policy),
                         JS_MEMBER(enable_visualization),
                         JS_MEMBER(enable_blocking_mode));
};

} // namespace redoxi_works::samples_nodes::drivers