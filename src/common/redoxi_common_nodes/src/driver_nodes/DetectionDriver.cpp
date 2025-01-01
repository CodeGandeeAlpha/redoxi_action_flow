#include <redoxi_common_nodes/_pch.hpp>

#include <redoxi_common_nodes/driver_nodes/DetectionDriver.hpp>
#include <redoxi_common_cpp/ros_utils/shm_utils.hpp>

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
    output_source_data.frame_data = callee_request.get_source_data().get_primary_frame();
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
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Processing input request", msg_uuid_str);

    const auto &primary_frame = source_data->get_goal()->frame_bundle.primary_frame;
    image_utils::FrameMediator fm(&primary_frame);

    // do we have shared memory?
    {
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Checking frame shm token readability", msg_uuid_str);
        auto is_readable = shm_utils::ShmTokenTraits::is_readable_by_default_client(primary_frame.shm_token);
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Frame shm token readability: {}",
                     msg_uuid_str, is_readable ? "true" : "false");
    }

    const auto frame_number = fm.get_frame_number();
    const auto source_frame_index = fm.get_source_frame_index();
    const auto source_frame_timestamp = fm.get_source_timestamp_flat();
    const auto source_image_encoding = fm.get_encoding();
    const auto signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Driver got frame number={}, source_frame_index={}, source_frame_timestamp={}, source_image_encoding={}, signal code={}",
                 msg_uuid_str, frame_number, source_frame_index,
                 fmt::format("{:.6f} sec", source_frame_timestamp.count() / 1e6),
                 source_image_encoding, control_signal_code_to_string(signal_code));

    // fill source data
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Creating request from source data", msg_uuid_str);
    CalleeTypes::RequestOutputSourceData_t output_source_data;
    output_source_data.from_frame_bundle(source_data->get_goal()->frame_bundle);
    output_request->set_source_data(output_source_data);

    // get signal code and show it
    // auto signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    // output_request->set_control_signal_code(signal_code);
    // RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Set control signal code to {}",
    //              msg_uuid_str, control_signal_code_to_string(signal_code));

    // const auto &raw_image = output_request->get_source_data().get_primary_frame().get_data_as<image_ports::types::FrameWithMetadata::Frame_t>().raw_image;
    // RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] sending raw image data size={}, encoding={}",
    //              msg_uuid_str, raw_image.data.size(), raw_image.encoding);

    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Done, created request from source data", msg_uuid_str);

    return 0;
}

} // namespace redoxi_works::common_nodes::drivers

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(redoxi_works::common_nodes::drivers::DetectionDriver)
