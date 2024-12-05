#pragma once

#include <universal_mot_trackers/visibility_control.h>
#include <redoxi_common_nodes/detection_ports/DetectionResponseInputPort.hpp>
#include <redoxi_common_nodes/detection_ports/DetectionRequestOutputPort.hpp>
#include <redoxi_common_nodes/image_ports/AsyncImageOutputPort.hpp>
#include <universal_mot_trackers/UniversalMotTrackerNode.hpp>
#include <redoxi_common_nodes/driver_nodes/CallActionDriverBase.hpp>

namespace redoxi_works::model_nodes::universal_mot_trackers
{
using TrackerDriverRequestPort = detection_ports::response_only::DetectionResponseInputPort;
using TrackerDriverCalleePort = detection_ports::request_response::DetectionRequestOutputPort;
using TrackerDriverOutputPort = image_ports::AsyncImageOutputPort;

class TrackerDriverNode : public common_nodes::drivers::CallActionDriverBase<
                              TrackerDriverRequestPort,
                              TrackerDriverCalleePort,
                              TrackerDriverOutputPort>
{
  public:
    using BaseNode_t = common_nodes::drivers::CallActionDriverBase<TrackerDriverRequestPort, TrackerDriverCalleePort, TrackerDriverOutputPort>;
    using BaseInitConfig_t = BaseNode_t::InitConfig_t;
    using BaseRuntimeConfig_t = BaseNode_t::RuntimeConfig_t;
    using BaseNode_t::CallActionDriverBase;

  protected:
    virtual int _on_process_input_request(InputRequestHandler_t::OutputRequest_t *output_request,
                                          std::optional<InputRequestHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
                                          InputRequestHandler_t::InputActionResult_t *output_result,
                                          std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                          InputRequestHandler_t::ResourceToken_t &resource_token) override;

    virtual int _on_process_callee_result(OutputTypes::OutputRequest_t *output_request,
                                          OutputTypes::OutputDeliveryPolicy_t *output_enqueue_policy,
                                          std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                          const CalleeTypes::RequestOutputRequest_t &callee_request,
                                          const CalleeTypes::Downstream_t &downstream) override;
};
} // namespace redoxi_works::model_nodes::universal_mot_trackers