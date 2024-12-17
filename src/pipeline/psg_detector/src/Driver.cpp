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

    RDX_INFO_DEV(this, __func__, false, "{}", "开始处理检测结果");

    OutputTypes::OutputSourceData_t output_pipeline_source_data;
    // 设置auxiliary_data的类型，用于可视化
    output_pipeline_source_data.auxiliary_data = std::string("detector");

    RDX_INFO_DEV(this, __func__, false, "{}", "从callee请求中获取文档数据");
    auto document = callee_request.get_source_data().get_document();

    RDX_INFO_DEV(this, __func__, false, "{}", "更新文档中的检测结果");
    document.detections = callee_result->detections;

    RDX_INFO_DEV(this, __func__, false, "{}", "设置输出数据");
    output_pipeline_source_data.set_document(document);
    output_request->set_source_data(output_pipeline_source_data);

    RDX_INFO_DEV(this, __func__, false, "{}", "设置控制信号");
    const auto signal_code = callee_request.get_control_signal_code();
    output_request->set_control_signal_code(signal_code);

    RDX_INFO_DEV(this, __func__, false, "{}", "处理完成");
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

    RDX_INFO_DEV(this, __func__, false, "{}", "开始处理输入请求");

    //! 从输入数据中获取消息UUID
    auto msg_uuid = InputTypes::ActionDataTrait_t::get_uuid(*source_data->get_goal());
    auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] 正在从源数据创建请求", msg_uuid_str);

    //! 设置输出源数据
    CalleeTypes::RequestOutputSourceData_t output_source_data;
    output_source_data.set_document(source_data->get_goal()->document);
    RDX_INFO_DEV(this, __func__, false, "{}", "已设置输出源数据");

    //! 获取目标句柄和控制信号
    auto goal_handle = source_data->get_goal_handle_future().get();
    auto control_signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    RDX_INFO_DEV(this, __func__, true,
                 "处理输入数据：帧号 {}, 控制信号代码 {}",
                 source_data->get_goal()->document.frame_bundle.primary_frame.metadata.frame_num,
                 int(control_signal_code));

    //! 创建传输请求
    RDX_INFO_DEV(this, __func__, false, "{}", "开始创建传输请求");
    output_request->set_source_data(output_source_data);
    const auto signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    output_request->set_control_signal_code(signal_code);

    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] 已设置控制信号代码为 {}",
                 msg_uuid_str, control_signal_code_to_string(signal_code));

    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] 完成，已从源数据创建请求", msg_uuid_str);

    return 0;
}

} // namespace redoxi_works