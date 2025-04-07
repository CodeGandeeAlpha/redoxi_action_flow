#pragma once

#include <redoxi_common_nodes/driver_nodes/CallActionDriverBase.hpp>
#include <redoxi_common_nodes/image_ports/AsyncImageInputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionRequestOutputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionResponseOutputPort.hpp>

namespace redoxi_works::common_nodes::drivers
{
class DetectionDriver : public CallActionDriverBase<image_ports::AsyncImageInputPort,
                                                    detection_ports::request_response::DetectionRequestOutputPort,
                                                    detection_ports::response_only::DetectionResponseOutputPort>
{
  public:
    using BaseNode_t = CallActionDriverBase<image_ports::AsyncImageInputPort,
                                            detection_ports::request_response::DetectionRequestOutputPort,
                                            detection_ports::response_only::DetectionResponseOutputPort>;
    DetectionDriver(const std::string &name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : BaseNode_t(name, options)
    {
    }

    DetectionDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : DetectionDriver("DetectionDriver", options)
    {
    }


  protected:
    int _on_process_callee_result(OutputTypes::OutputRequest_t *output_request,
                                  OutputTypes::OutputDeliveryPolicy_t *output_enqueue_policy,
                                  std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                  const CalleeTypes::RequestOutputRequest_t &callee_request,
                                  const CalleeTypes::Downstream_t &downstream) override;

    int _on_process_input_request(InputRequestHandler_t::OutputRequest_t *output_request,
                                  std::optional<InputRequestHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
                                  InputRequestHandler_t::InputActionResult_t *output_result,
                                  std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                  InputRequestHandler_t::ResourceToken_t &resource_token) override;
};
} // namespace redoxi_works::common_nodes::drivers