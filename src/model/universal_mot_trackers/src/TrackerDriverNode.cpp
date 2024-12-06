#include <universal_mot_trackers/TrackerDriverNode.hpp>

namespace redoxi_works::model_nodes::universal_mot_trackers
{

int TrackerDriverNode::_on_process_input_request(CalleeTypes::RequestOutputRequest_t *out_callee_request,
                                                 std::optional<CalleeTypes::RequestOutputDeliveryPolicy_t> *out_callee_enqueue_policy,
                                                 InputTypes::ActionResult_t *out_upstream_result,
                                                 std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                                 InputRequestHandler_t::ResourceToken_t &resource_token)
{
    using CalleeDataTrait_t = CalleeTypes::RequestOutputActionDataTrait_t;
    using RequestDataTrait_t = InputTypes::ActionDataTrait_t;

    const auto &detector_source_data = *source_data;
    auto &tracker_source_data = out_callee_request->get_source_data();

    // fill control code and task uid
    auto signal_code = RequestDataTrait_t::get_control_signal_code(*detector_source_data.get_goal());
    out_callee_request->set_control_signal_code(signal_code);

    // get detections from source data, and put them into tracker source data
    const auto &det_msg = detector_source_data.get_goal()->detections;
    // tracker_source_data


    return 0;
}

int TrackerDriverNode::_on_process_callee_result(OutputTypes::OutputRequest_t *out_downstream_request,
                                                 OutputTypes::OutputDeliveryPolicy_t *out_downstream_enqueue_policy,
                                                 std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                                 const CalleeTypes::RequestOutputRequest_t &callee_request,
                                                 const CalleeTypes::Downstream_t &downstream)
{
    return 0;
}

} // namespace redoxi_works::model_nodes::universal_mot_trackers
