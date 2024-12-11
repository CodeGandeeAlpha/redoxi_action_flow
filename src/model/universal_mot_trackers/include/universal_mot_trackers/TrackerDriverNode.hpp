#pragma once

#include <universal_mot_trackers/visibility_control.h>
#include <redoxi_common_nodes/detection_ports/DetectionResponseInputPort.hpp>
// #include <redoxi_common_nodes/detection_ports/DetectionRequestOutputPort.hpp>
#include <universal_mot_trackers/tracking_ports/TrackingRequestOutputPort.hpp>
#include <redoxi_common_nodes/image_ports/AsyncImageOutputPort.hpp>
#include <universal_mot_trackers/UniversalMotTrackerNode.hpp>
#include <redoxi_common_nodes/driver_nodes/CallActionDriverBase.hpp>

namespace redoxi_works::model_nodes::universal_mot_trackers
{

// computed detections -> tracking request -> (tracked result) -> image output
using TrackerDriverRequestPort = detection_ports::response_only::DetectionResponseInputPort;
using TrackerDriverCalleePort = tracking_ports::request_response::TrackingRequestOutputPort;
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
    using BaseNode_t::BaseNode_t;

  protected:
    int _on_process_input_request(CalleeTypes::RequestOutputRequest_t *out_callee_request,
                                  std::optional<CalleeTypes::RequestOutputDeliveryPolicy_t> *out_callee_enqueue_policy,
                                  InputTypes::ActionResult_t *out_upstream_result,
                                  std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                  InputRequestHandler_t::ResourceToken_t &resource_token) override;

    int _on_process_callee_result(OutputTypes::OutputRequest_t *out_downstream_request,
                                  OutputTypes::OutputDeliveryPolicy_t *out_downstream_enqueue_policy,
                                  std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                  const CalleeTypes::RequestOutputRequest_t &callee_request,
                                  const CalleeTypes::Downstream_t &downstream) override;
};
} // namespace redoxi_works::model_nodes::universal_mot_trackers