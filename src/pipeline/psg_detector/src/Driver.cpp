#include <psg_detector/Driver.hpp>

namespace redoxi_works
{

int PSGDetectorDriver::_on_process_callee_result(OutputTypes::OutputRequest_t *output_request,
                                                 OutputTypes::OutputDeliveryPolicy_t *output_enqueue_policy,
                                                 std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                                 const CalleeTypes::RequestOutputRequest_t &callee_request,
                                                 const CalleeTypes::Downstream_t &downstream)
{
    (void)downstream;
    (void)output_enqueue_policy;

    OutputTypes::OutputSourceData_t output_pipeline_source_data;
    auto document = callee_request.get_source_data().get_document();
    document.detections = callee_result->detections;
    output_pipeline_source_data.set_document(document);
    output_request->set_source_data(output_pipeline_source_data);

    const auto signal_code = callee_request.get_control_signal_code();
    output_request->set_control_signal_code(signal_code);
    return 0;
}

int PSGDetectorDriver::_on_process_input_request(InputRequestHandler_t::OutputRequest_t *output_request,
                                                 std::optional<InputRequestHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
                                                 InputRequestHandler_t::InputActionResult_t *output_result,
                                                 std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                                 InputRequestHandler_t::ResourceToken_t &resource_token)
{
    (void)resource_token;
    (void)output_result;
    (void)output_enqueue_policy;

    // from input source data to output source data
    auto msg_uuid = InputTypes::ActionDataTrait_t::get_uuid(*source_data->get_goal());
    auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Creating request from source data", msg_uuid_str);
    CalleeTypes::RequestOutputSourceData_t output_source_data;
    output_source_data.set_document(source_data->get_goal()->document);

    auto goal_handle = source_data->get_goal_handle_future().get();
    auto control_signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    RDX_INFO_DEV(this, __func__, true,
                 "on_process_input_data()中frame num: {}, control signal code: {}",
                 source_data->get_goal()->document.frame_bundle.primary_frame.metadata.frame_num, int(control_signal_code));


    // create delivery request
    output_request->set_source_data(output_source_data);
    const auto signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    output_request->set_control_signal_code(signal_code);


    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Set control signal code to {}",
                 msg_uuid_str, control_signal_code_to_string(signal_code));

    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Done, created request from source data", msg_uuid_str);

    return 0;
}

} // namespace redoxi_works