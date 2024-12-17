#include <psg_pose_detector/Driver.hpp>

namespace redoxi_works
{

int PSGPoseDetectorDriver::_on_process_callee_result(OutputTypes::OutputRequest_t *output_request,
                                                     OutputTypes::OutputDeliveryPolicy_t *output_enqueue_policy,
                                                     std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                                     const CalleeTypes::RequestOutputRequest_t &callee_request,
                                                     const CalleeTypes::Downstream_t &downstream)
{
    (void)downstream;
    (void)output_enqueue_policy;

    OutputTypes::OutputSourceData_t output_pipeline_source_data;
    // 根据frame_number获取document
    RDX_LOG_DEBUG(this, __func__, "{}", "开始从document map中获取document");
    auto document = m_document_map.synchronize()->at(callee_request.get_source_data().get_frame_bundle().primary_frame.metadata.frame_num);
    // 删掉字典中的document
    RDX_LOG_DEBUG(this, __func__, "{}", "开始从document map中删除document");
    m_document_map.synchronize()->erase(callee_request.get_source_data().get_frame_bundle().primary_frame.metadata.frame_num);

    if (callee_result->keypoints.size() > 0) {
        RDX_LOG_DEBUG(this, __func__, "{}", "开始处理keypoints结果");
        if (callee_result->is_matched_by_uid) { // 如果是基于x_group_id匹配的
            RDX_LOG_DEBUG(this, __func__, "{}", "基于x_group_id匹配keypoints");
            for (size_t i = 0; i < callee_result->keypoints.size(); ++i) {
                for (size_t j = 0; j < document->detections.size(); ++j) {
                    if (document->detections[j].x_uid == callee_result->keypoints[i].x_group_uid) {
                        document->detections[j].keypoints = callee_result->keypoints[i];
                        break;
                    }
                }
            }
        } else { // 如果是按顺序保存的
            RDX_LOG_DEBUG(this, __func__, "{}", "按顺序保存keypoints");
            for (size_t i = 0; i < callee_result->keypoints.size(); ++i) {
                document->detections[callee_request.get_source_data().get_detections_indices()[i]].keypoints = callee_result->keypoints[i];
            }
        }
    }
    output_pipeline_source_data.set_document(*document);
    output_request->set_source_data(output_pipeline_source_data);
    const auto signal_code = callee_request.get_control_signal_code();
    output_request->set_control_signal_code(signal_code);
    return 0;
}

int PSGPoseDetectorDriver::_on_process_input_request(InputRequestHandler_t::OutputRequest_t *output_request,
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

    // 将document数据放入document map中
    m_document_map.synchronize()->insert({source_data->get_goal()->document.frame_bundle.primary_frame.metadata.frame_num,
                                          std::make_shared<psg_private_msgs::msg::PsgDocument>(source_data->get_goal()->document)});

    //! 从输入数据创建输出数据
    CalleeTypes::RequestOutputSourceData_t output_source_data;
    output_source_data.set_frame_bundle(source_data->get_goal()->document.frame_bundle);
    output_source_data.set_detections(source_data->get_goal()->document.detections);

    //! 根据种类挑选出body的detections，并记录其在document中的索引
    RDX_INFO_DEV(this, __func__, true, "{}", "开始筛选body类型的检测框");
    std::vector<size_t> body_detections_indices;
    for (size_t i = 0; i < source_data->get_goal()->document.detections.size(); ++i) {
        if (source_data->get_goal()->document.detections[i].category == 0) { // 0: body, 1: head, 2: face
            body_detections_indices.push_back(i);
        }
    }
    RDX_INFO_DEV(this, __func__, true, "找到{}个body检测框", body_detections_indices.size());
    output_source_data.set_detections_indices(body_detections_indices);

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