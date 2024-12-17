#include <psg_tracker/Driver.hpp>

namespace redoxi_works
{

int PSGTrackerDriver::_on_process_callee_result(OutputTypes::OutputRequest_t *output_request,
                                                OutputTypes::OutputDeliveryPolicy_t *output_enqueue_policy,
                                                std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                                const CalleeTypes::RequestOutputRequest_t &callee_request,
                                                const CalleeTypes::Downstream_t &downstream)
{
    (void)downstream;
    (void)output_enqueue_policy;

    OutputTypes::OutputSourceData_t output_pipeline_source_data;
    // 设置auxiliary_data的类型，用于可视化
    output_pipeline_source_data.auxiliary_data = std::string("tracker");
    // 根据frame_number获取document
    RDX_LOG_DEBUG(this, __func__, false, "开始从document map中获取document", 0);
    auto document = m_document_map.synchronize()->at(callee_request.get_source_data().get_frame_bundle().primary_frame.metadata.frame_num);
    // 删掉字典中的document
    RDX_LOG_DEBUG(this, __func__, false, "开始从document map中删除document", 0);
    m_document_map.synchronize()->erase(callee_request.get_source_data().get_frame_bundle().primary_frame.metadata.frame_num);

    RDX_LOG_DEBUG(this, __func__, false, "开始处理track_targets结果", 0);
    for (const auto &track_target : callee_result->track_targets) {
        RDX_LOG_DEBUG(this, __func__, false,
                      "处理track_target - track_id:{}, track_status:{}, uuid:{}",
                      track_target.track_id,
                      track_target.track_status.bitmask,
                      boost::uuids::to_string(to_boost_uuid(track_target.x_group_uid.uuid)));

        // 1. 将track_targets中的track_id和person_id进行匹配，并赋予跟踪的id
        {
            auto lock_ptr_person_map = m_person_map.synchronize();
            if (lock_ptr_person_map->find(track_target.x_group_uid.uuid) != lock_ptr_person_map->end()) {
                auto &person = (*lock_ptr_person_map)[track_target.x_group_uid.uuid];
                person->track_id = track_target.track_id;
                RDX_LOG_DEBUG(this, __func__, false,
                              "更新person track_id - uuid:{}, track_id:{}",
                              boost::uuids::to_string(to_boost_uuid(track_target.x_group_uid.uuid)),
                              track_target.track_id);
                person->body = track_target.predicted_detection;
            }
        }

        // 2. 将track_target中的track_id写入document中的persons
        for (auto &person : document->persons) {
            if (person.x_uid == track_target.x_group_uid) {
                person.track_id = track_target.track_id;
                person.body = track_target.predicted_detection;
                break;
            }
        }

        // 3. 收集closed trajectory，并写入document中的trajectories
        // if track_target is new, create a new trajectory
        if (track_target.track_status.bitmask & redoxi_public_msgs::msg::TrackObjectStatus::NEW_BIT) {
            m_closed_trajectory_map[track_target.track_id] = std::vector<PSGTrackerDriver::ArrayUUID>();
            m_closed_trajectory_map[track_target.track_id].push_back(track_target.x_group_uid.uuid);
            RDX_LOG_DEBUG(this, __func__, false,
                          "创建新轨迹 - track_id:{}, uuid:{}",
                          track_target.track_id,
                          boost::uuids::to_string(to_boost_uuid(track_target.x_group_uid.uuid)));
        }
        // if track_target is open, add it to trajectory
        else if (track_target.track_status.bitmask & redoxi_public_msgs::msg::TrackObjectStatus::OPEN_BIT) {
            m_closed_trajectory_map[track_target.track_id].push_back(track_target.x_group_uid.uuid);
            RDX_LOG_DEBUG(this, __func__, false,
                          "添加到现有轨迹 - track_id:{}, uuid:{}",
                          track_target.track_id,
                          boost::uuids::to_string(to_boost_uuid(track_target.x_group_uid.uuid)));
        }
        // if track_target is close, get trajectory and remove it from buffer
        else if (track_target.track_status.bitmask & redoxi_public_msgs::msg::TrackObjectStatus::CLOSE_BIT) {
            // get closed trajectory uuids
            auto closed_trajectory_uuids = m_closed_trajectory_map[track_target.track_id];
            // remove closed trajectory from buffer
            m_closed_trajectory_map.erase(track_target.track_id);
            // get closed trajectory
            psg_private_msgs::msg::PersonTrajectory closed_trajectory;
            closed_trajectory.track_id = track_target.track_id;
            {
                auto lock_ptr_person_map = m_person_map.synchronize();
                for (auto &uuid : closed_trajectory_uuids) {
                    closed_trajectory.persons.push_back(*(*lock_ptr_person_map)[uuid]);
                    if (lock_ptr_person_map->find(uuid) != lock_ptr_person_map->end()) {
                        lock_ptr_person_map->erase(uuid);
                    }
                }
            }
            // put closed trajectory to document
            document->trajectories.push_back(closed_trajectory);
            RDX_LOG_DEBUG(this, __func__, false,
                          "关闭轨迹 - track_id:{}, 轨迹长度:{}",
                          track_target.track_id,
                          closed_trajectory.persons.size());
        } else
            continue;
    }
    output_pipeline_source_data.set_document(*document);
    output_request->set_source_data(output_pipeline_source_data);

    // create delivery request
    const auto signal_code = callee_request.get_control_signal_code();
    output_request->set_control_signal_code(signal_code);
    return 0;
}

int PSGTrackerDriver::_on_process_input_request(InputRequestHandler_t::OutputRequest_t *output_request,
                                                std::optional<InputRequestHandler_t::OutputDeliveryPolicy_t> *output_enqueue_policy,
                                                InputRequestHandler_t::InputActionResult_t *output_result,
                                                std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                                InputRequestHandler_t::ResourceToken_t &resource_token)
{
    (void)resource_token;
    (void)output_result;
    (void)output_enqueue_policy;

    // 将document数据放入document map中
    auto msg_uuid = InputTypes::ActionDataTrait_t::get_uuid(*source_data->get_goal());
    auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Creating request from source data", msg_uuid_str);

    RDX_INFO_DEV(this, __func__, true, "{}", "正在将document数据插入document map...");
    m_document_map.synchronize()->insert({source_data->get_goal()->document.frame_bundle.primary_frame.metadata.frame_num,
                                          std::make_shared<psg_private_msgs::msg::PsgDocument>(source_data->get_goal()->document)});
    RDX_INFO_DEV(this, __func__, true, "{}", "document数据插入完成");

    // 将person数据放入person map中
    RDX_INFO_DEV(this, __func__, true, "{}", "正在将person数据插入person map...");
    auto lock_ptr_person_map = m_person_map.synchronize();
    for (const auto &person : source_data->get_goal()->document.persons) {
        lock_ptr_person_map->insert({person.x_uid.uuid, std::make_shared<psg_private_msgs::msg::Person>(person)});
    }
    RDX_INFO_DEV(this, __func__, true, "{}", "person数据插入完成");

    // 创建delivery request，并推送到output port model
    RDX_INFO_DEV(this, __func__, true, "{}", "正在创建output source data...");
    // from input source data to output source data
    CalleeTypes::RequestOutputSourceData_t output_source_data;
    output_source_data.set_frame_bundle(source_data->get_goal()->document.frame_bundle);
    output_source_data.set_persons(source_data->get_goal()->document.persons);
    RDX_INFO_DEV(this, __func__, true, "{}", "output source data创建完成");

    RDX_INFO_DEV(this, __func__, true, "{}", "正在获取goal handle和control signal code...");
    auto goal_handle = source_data->get_goal_handle_future().get();
    auto control_signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    RDX_INFO_DEV(this, __func__, true,
                 "on_process_input_data()中frame num: {}, control signal code: {}",
                 source_data->get_goal()->document.frame_bundle.primary_frame.metadata.frame_num, int(control_signal_code));

    // create delivery request
    RDX_INFO_DEV(this, __func__, true, "{}", "正在创建delivery request...");
    output_request->set_source_data(output_source_data);
    const auto signal_code = InputTypes::ActionDataTrait_t::get_control_signal_code(*source_data->get_goal());
    output_request->set_control_signal_code(signal_code);
    RDX_INFO_DEV(this, __func__, true, "{}", "delivery request创建完成");


    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Set control signal code to {}",
                 msg_uuid_str, control_signal_code_to_string(signal_code));

    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Done, created request from source data", msg_uuid_str);

    return 0;
}

} // namespace redoxi_works