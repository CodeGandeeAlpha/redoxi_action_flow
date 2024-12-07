#include <redoxi_common_nodes/driver_nodes/DetectionDriver.hpp>

namespace redoxi_works::common_nodes::drivers
{

int DetectionDriver::_on_process_callee_result(OutputTypes::OutputRequest_t *output_request,
                                               OutputTypes::OutputDeliveryPolicy_t *output_enqueue_policy,
                                               std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                               const CalleeTypes::RequestOutputRequest_t &callee_request,
                                               const CalleeTypes::Downstream_t &downstream)
{
    (void)downstream;
    (void)output_enqueue_policy;

    OutputTypes::OutputSourceData_t output_source_data;
    output_source_data.detections = callee_result->detections;
    output_source_data.image = callee_request.get_source_data().get_image();
    output_source_data.image_encoding = callee_request.get_source_data().get_image_encoding();
    output_source_data.uid = callee_request.get_source_data().get_uuid();
    output_request->set_source_data(output_source_data);
    return 0;
}

int DetectionDriver::_on_process_input_request(InputRequestHandler_t::OutputRequest_t *output_request,
                                               std::optional<InputRequestHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
                                               InputRequestHandler_t::InputActionResult_t *output_result,
                                               std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                               InputRequestHandler_t::ResourceToken_t &resource_token)
{
    (void)resource_token;
    (void)output_result;
    (void)output_enqueue_policy;

    auto msg_uuid = InputTypes::ActionDataTrait_t::get_uuid(*source_data->get_goal());
    auto msg_uuid_str = boost::uuids::to_string(msg_uuid);

    const auto frame_number = source_data->get_goal()->frame.metadata.frame_num;
    const auto source_frame_index = source_data->get_goal()->frame.metadata.source_frame_index;
    const auto source_frame_timestamp = source_data->get_goal()->frame.metadata.source_timestamp;
    const auto source_image_encoding = source_data->get_goal()->frame.metadata.encoding;
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Driver got frame number={}, source_frame_index={}, source_frame_timestamp={}, source_image_encoding={}",
                 msg_uuid_str, frame_number, source_frame_index,
                 fmt::format("{}.{:06d} sec", source_frame_timestamp.sec, source_frame_timestamp.nanosec),
                 source_image_encoding);

    // create request
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Creating request from source data", msg_uuid_str);
    CalleeTypes::RequestOutputSourceData_t output_source_data;
    const auto &raw_image = source_data->get_goal()->frame.raw_image;
    if (!raw_image.data.empty()) {
        cv::Mat image;
        image = cv_bridge::toCvCopy(raw_image, raw_image.encoding)->image;

        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Extracted image from source data, width={}, height={}",
                     msg_uuid_str, image.cols, image.rows);
        output_source_data.set_image(image, raw_image.encoding);
    }

    output_source_data.set_frame_metadata(source_data->get_goal()->frame.metadata);
    output_request->set_source_data(output_source_data);

    // get signal code and show it
    auto signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    output_request->set_control_signal_code(signal_code);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Set control signal code to {}",
                 msg_uuid_str, control_signal_code_to_string(signal_code));

    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Done, created request from source data", msg_uuid_str);

    return 0;
}

} // namespace redoxi_works::common_nodes::drivers