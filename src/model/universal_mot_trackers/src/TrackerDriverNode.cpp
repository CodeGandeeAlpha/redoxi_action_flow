#include <universal_mot_trackers/TrackerDriverNode.hpp>

namespace redoxi_works::model_nodes::universal_mot_trackers
{

int TrackerDriverNode::_on_process_input_request(InputRequestHandler_t::OutputRequest_t *output_request,
                                                 std::optional<InputRequestHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
                                                 InputRequestHandler_t::InputActionResult_t *output_result,
                                                 std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                                 InputRequestHandler_t::ResourceToken_t &resource_token)
{
    using CalleeDataTrait_t = CalleeTypes::RequestOutputActionDataTrait_t;
    using RequestDataTrait_t = InputTypes::ActionDataTrait_t;

    auto &callee_source_data = output_request->get_source_data();

    // fill control code and task uid
    auto signal_code = RequestDataTrait_t::get_control_signal_code(*source_data->get_goal());
    output_request->set_control_signal_code(signal_code);

    return 0;
}

int TrackerDriverNode::_on_process_callee_result(OutputTypes::OutputRequest_t *output_request,
                                                 OutputTypes::OutputDeliveryPolicy_t *output_enqueue_policy,
                                                 std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                                 const CalleeTypes::RequestOutputRequest_t &callee_request,
                                                 const CalleeTypes::Downstream_t &downstream)
{
    return 0;
}

} // namespace redoxi_works::model_nodes::universal_mot_trackers
