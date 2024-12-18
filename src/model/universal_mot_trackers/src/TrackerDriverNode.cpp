#include <universal_mot_trackers/TrackerDriverNode.hpp>

namespace redoxi_works::model_nodes::universal_mot_trackers
{

int TrackerDriverNode::_on_process_input_request(CalleeTypes::RequestOutputRequest_t *out_callee_request,
                                                 std::optional<CalleeTypes::RequestOutputDeliveryPolicy_t> *out_callee_enqueue_policy,
                                                 InputTypes::ActionResult_t *out_upstream_result,
                                                 std::shared_ptr<const InputTypes::SourceData_t> source_data,
                                                 InputRequestHandler_t::ResourceToken_t &resource_token)
{
    (void)out_upstream_result;
    (void)resource_token;
    (void)out_callee_enqueue_policy;

    // using CalleeDataTrait_t = CalleeTypes::RequestOutputActionDataTrait_t;
    using RequestDataTrait_t = InputTypes::ActionDataTrait_t;

    const auto &detector_source_data = *source_data;
    auto &tracker_source_data = out_callee_request->get_source_data();

    auto msg_uuid = RequestDataTrait_t::get_uuid(*detector_source_data.get_goal());
    RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Sending detections to tracker",
                 UUIDTrait::to_string(msg_uuid));

    // fill control code and task uid
    auto signal_code = RequestDataTrait_t::get_control_signal_code(*detector_source_data.get_goal());
    out_callee_request->set_control_signal_code(signal_code);
    out_callee_request->set_source_task_metadata(RequestDataTrait_t::get_source_task_metadata(*detector_source_data.get_goal()));

    // get detections from source data, and put them into tracker source data
    const auto &det_msg = detector_source_data.get_goal()->detections;
    tracker_source_data.set_detections(det_msg);
    auto &frame_data = tracker_source_data.get_primary_frame();
    frame_data.from_frame_msg(detector_source_data.get_goal()->frame_bundle.primary_frame);

    // track the uuid
    tracker_source_data.set_uuid(msg_uuid);

    RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Tracker request data is ready",
                 UUIDTrait::to_string(msg_uuid));

    return 0;
}

int TrackerDriverNode::_on_process_callee_result(OutputTypes::OutputRequest_t *out_downstream_request,
                                                 OutputTypes::OutputDeliveryPolicy_t *out_downstream_enqueue_policy,
                                                 std::shared_ptr<const CalleeTypes::RequestOutputActionResult_t> callee_result,
                                                 const CalleeTypes::RequestOutputRequest_t &callee_request,
                                                 const CalleeTypes::Downstream_t &downstream)
{
    (void)out_downstream_enqueue_policy;
    (void)downstream;

    // copy control signal code and source task metadata
    out_downstream_request->copy_meta_info_from(callee_request);

    // get the source data
    auto &ds_source_data = out_downstream_request->get_source_data();
    auto msg_uuid = callee_request.get_source_data().get_uuid();
    RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Got tracked result",
                 UUIDTrait::to_string(msg_uuid));

    // get all tracked targets, and pass them to output port
    auto &track_targets = callee_result->track_targets;
    ds_source_data.set_track_targets(track_targets);

    // also pass the frame
    auto &frame_data = callee_request.get_source_data().get_primary_frame();
    ds_source_data.set_primary_frame(frame_data);

    return 0;
}

cv::Mat TrackerDriverNode::_draw_tracked_result(const CalleeTypes::RequestOutputActionResult_t &callee_result,
                                                const CalleeTypes::RequestOutputRequest_t &callee_request,
                                                const CalleeTypes::Downstream_t &downstream)
{
    (void)downstream;
    // get the source data
    auto msg_uuid = callee_request.get_source_data().get_uuid();
    RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Drawing tracked result",
                 UUIDTrait::to_string(msg_uuid));

    // get the tracked result and draw it on the image
    const auto &input_frame = callee_request.get_source_data().get_primary_frame();
    auto fm = input_frame.to_frame_mediator();
    auto canvas = fm.to_cv_image_copy();

    // draw the tracked result
    std::vector<redoxi_public_msgs::msg::Detection> dets;
    for (const auto &track_target : callee_result.track_targets) {
        auto det = track_target.predicted_detection;

        // FIXME: using track_id as category is not good but works for now
        // det.category = track_target.track_id;
        // det.confidence = track_target.confidence;
        det.semantic_identity.value = track_target.track_id;
        det.semantic_identity.name = "track_" + std::to_string(track_target.track_id);
        det.semantic_identity.confidence = track_target.confidence;
        dets.push_back(det);
    }

    RDX_INFO_DEV(nullptr, __func__, true, "[msg_uuid={}] Drawing tracked result, {} detections",
                 UUIDTrait::to_string(msg_uuid), dets.size());
    image_utils::DrawDetectionsOptions draw_opts;
    draw_opts.colorization_mode = decltype(draw_opts.colorization_mode)::SemanticIdentity;
    image_utils::draw_detections(&canvas, dets, draw_opts);

    return canvas;
}

} // namespace redoxi_works::model_nodes::universal_mot_trackers
