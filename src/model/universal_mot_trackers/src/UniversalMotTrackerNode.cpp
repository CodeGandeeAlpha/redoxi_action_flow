#include <universal_mot_trackers/UniversalMotTrackerNode.hpp>
#include <universal_mot_trackers/UniversalMotTrackerImpl.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <map>

namespace rxt = RedoxiTrack;

namespace redoxi_works::model_nodes::universal_mot_trackers
{
UniversalMotTrackerNode::UniversalMotTrackerNode(const std::string &name, const rclcpp::NodeOptions &options)
    : common_nodes::OpenCloseNode(name, options)
{
    m_impl = std::make_shared<Impl>();
}

int UniversalMotTrackerNode::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
{
    auto init_config = std::static_pointer_cast<InitConfig_t>(config);

    // create port
    if (init_config->input_port_config && !init_config->input_port_config->get_action_name().empty()) {
        RDX_INFO_DEV(this, __func__, "{}", "Creating port");
        m_input_port = std::make_shared<InputPort_t>(this);
        auto ret = m_input_port->init(init_config->input_port_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("{}", "Failed to create port");
        }
    } else {
        RDX_INFO_DEV(this, __func__, "{}", "No input port config, skipping port creation");
    }

    // create publisher for visualization
    if (!init_config->publish_visualization_topic.empty()) {
        RDX_INFO_DEV(this, __func__, "{}", "Creating publisher for visualization");
        m_pub_visualization = std::make_shared<StampedImagePub>(this, init_config->publish_visualization_topic);
    }

    // create publisher for performance probe
    if (!init_config->publish_probe_topic.empty()) {
        RDX_INFO_DEV(this, __func__, "{}", "Creating publisher for probe");
        m_pub_probe = this->create_publisher<std_msgs::msg::String>(init_config->publish_probe_topic,
                                                                    DefaultParams::get_probe_publisher_qos());
    }

    return 0;
}

int UniversalMotTrackerNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
{
    (void)config;
    // auto runtime_config = std::static_pointer_cast<RuntimeConfig_t>(config);
    // auto init_config = std::static_pointer_cast<InitConfig_t>(m_init_config);
    // {
    //     auto ret = _create_input_port_handler(*init_config, *runtime_config);
    //     if (ret != 0) {
    //         RDX_RAISE_ERROR("{}", "Failed to create input port handler");
    //     }
    // }
    // nothing to do
    return 0;
}

int UniversalMotTrackerNode::_handle_input_data(InputPortHandler_t::InputActionResult_t *output_action_result,
                                                std::shared_ptr<InputPortHandler_t::InputSourceData_t> source_data,
                                                InputPortHandler_t::ResourceToken_t &resource_token)
{
    // perform tracking based on detections
    // note that, even if this is the end of tracking, we still need to check if there is any valid frame data
    // because it may still contains valid frame data

    RDX_INFO_DEV(this, __func__, "{}", "Handling input data");
    (void)resource_token;

    //! Check goal validity
    const auto *goal = source_data->get_goal();
    if (goal == nullptr) {
        RDX_RAISE_ERROR("{}", "Goal is not found, this is unexpected");
    }

    auto signal_code = InputActionDataTrait_t::get_control_signal_code(*goal);
    image_utils::FrameMediator fm_source(&source_data->get_goal()->frame_bundle.primary_frame);
    RDX_INFO_DEV(this, __func__, "Got frame number={}, signal code={}", fm_source.get_frame_number(), control_signal_code_to_string(signal_code));

    //! Convert detections to tracker format
    std::vector<rxt::DetectionPtr> track_detections;
    std::map<rxt::DetectionPtr, size_t> det2index; // invert index map, for quick lookup
    {
        RDX_INFO_DEV(this, __func__, "{}", "Converting detections to tracker format");
        for (size_t i = 0; i < goal->detections.size(); ++i) {
            auto &det = goal->detections[i];
            auto track_det = std::make_shared<rxt::SingleDetection>();
            track_det->set_bbox(rxt::BBOX(det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height));
            track_det->set_confidence(det.confidence);
            track_det->set_type(det.category);
            track_detections.push_back(track_det);
            det2index[track_det] = i;
        }
    }

    //! Setup event handlers
    RDX_INFO_DEV(this, __func__, "{}", "Setting up event handlers");
    auto event_handler = m_impl->m_tracker_impl->m_tracker_event_handler;

    // setup event handler to collect data from tracking events
    std::set<rxt::TrackTargetPtr> updated_tracked_targets;       // all tracked targets that have appeared in event callbacks
    std::map<rxt::TrackTargetPtr, rxt::DetectionPtr> target2det; // target to detection
    std::map<rxt::DetectionPtr, rxt::TrackTargetPtr> det2target; // detection to target
    event_handler->on_target_association_after =
        [&target2det,
         &det2target,
         &updated_tracked_targets](rxt::ExternalTrackingEventHandler::TrackerBase_t *sender,
                                   const rxt::ExternalTrackingEventHandler::TargetAssociation_t &evt_data) {
            // record which detection is associated to which target
            (void)sender;
            target2det[evt_data.m_target] = evt_data.m_detection;
            det2target[evt_data.m_detection] = evt_data.m_target;
            updated_tracked_targets.insert(evt_data.m_target);
            return 0;
        };
    event_handler->on_target_closed_after =
        [&updated_tracked_targets](rxt::ExternalTrackingEventHandler::TrackerBase_t *sender,
                                   const rxt::ExternalTrackingEventHandler::TargetClosed_t &evt_data) {
            (void)sender;
            updated_tracked_targets.erase(evt_data.m_target);
            return 0;
        };
    event_handler->on_target_created_after =
        [&target2det,
         &det2target,
         &updated_tracked_targets](rxt::ExternalTrackingEventHandler::TrackerBase_t *sender,
                                   const rxt::ExternalTrackingEventHandler::TargetAssociation_t &evt_data) {
            (void)sender;
            target2det[evt_data.m_target] = evt_data.m_detection;
            det2target[evt_data.m_detection] = evt_data.m_target;
            updated_tracked_targets.insert(evt_data.m_target);
            return 0;
        };
    event_handler->on_target_motion_predict_after =
        [&updated_tracked_targets](rxt::ExternalTrackingEventHandler::TrackerBase_t *sender,
                                   const rxt::ExternalTrackingEventHandler::TargetMotionPredict_t &evt_data) {
            (void)sender;
            updated_tracked_targets.insert(evt_data.m_target);
            return 0;
        };

    //! Check if we need to finish tracking
    auto tracker = m_impl->m_tracker_impl->m_tracker;
    bool end_of_tracking = signal_code == ControlSignalCode::Flush || signal_code == ControlSignalCode::Terminate;

    //! Extract input image
    RDX_INFO_DEV(this, __func__, "{}", "Extracting image");
    cv::Mat image;
    auto got_source_frame = fm_source.to_cv_image_copy(image) == 0;
    auto current_frame_number = fm_source.get_frame_number();
    auto tracking_frame_number = tracker->get_current_frame_number();
    RDX_INFO_DEV(this, __func__, "Report: current frame number={}, tracking frame number={}", current_frame_number, tracking_frame_number);

    // check if we should track this frame, track only if
    // 1. we got the source frame
    // 2. the frame number is valid (>=0)
    // 3. the frame number is greater than the current tracking frame number (ignore out-of-order frame)
    // 4. the image is valid
    bool is_trackable = got_source_frame && current_frame_number >= 0 && (current_frame_number > tracking_frame_number) && !image.empty();
    if (is_trackable) {
        RDX_INFO_DEV(this, __func__, "This frame (frame number={}) is trackable, do tracking", current_frame_number);
        if (tracking_frame_number == rxt::INIT_TRACKING_FRAME) {
            // init tracking
            RDX_INFO_DEV(this, __func__, "{}", "This is the first frame, initializing tracking");
            tracker->begin_track(image, track_detections, current_frame_number);
        } else {
            RDX_INFO_DEV(this, __func__, "{}", "This is subsequent frame, updating tracking");
            // update tracking
            tracker->track(image, track_detections, current_frame_number);
        }
    } else {
        if (!got_source_frame) {
            RDX_INFO_DEV(this, __func__, "{}", "Failed to get source frame, skip tracking");
        } else if (current_frame_number < 0) {
            RDX_INFO_DEV(this, __func__, "{}", "Invalid frame number, skip tracking");
        } else if (current_frame_number <= tracking_frame_number) {
            RDX_INFO_DEV(this, __func__, "Out-of-order frame (frame number={}, tracking frame number={}), skip tracking",
                         current_frame_number, tracking_frame_number);
        } else {
            RDX_INFO_DEV(this, __func__, "{}", "Invalid image, skip tracking");
        }
    }

    if (end_of_tracking) {
        RDX_INFO_DEV(this, __func__, "Received signal {}, finishing tracking", control_signal_code_to_string(signal_code));
        tracker->finish_track();
    }

    if (!is_trackable && !end_of_tracking) {
        RDX_WARN_PRODUCTION(this, __func__, "{}", "Unexpected, not trackable or end of tracking, skip tracking");
        return -1;
    }

    //! Collect tracking results
    RDX_INFO_DEV(this, __func__, "{}", "Collecting tracking results");
    const auto &all_open_targets = tracker->get_all_open_targets();

    // report track targets = all updated tracked targets + all remaining tracked targets
    // with all duplicates removed
    std::set<rxt::TrackTargetPtr> report_track_targets(updated_tracked_targets);
    for (const auto &target : all_open_targets) {
        report_track_targets.insert(target.second);
    }

    // filter the track targets to remove lost (unmatched) tracks
    // TODO: lost tracks are still useful in downstream, but it is defined differently in botsort and deepsort, so not usable now, add it back later
    std::vector<rxt::TrackTargetPtr> filtered_track_targets;
    for (const auto &target : report_track_targets) {
        if (!(target->get_path_state() & rxt::TrackPathStateBitmask::Lost)) {
            filtered_track_targets.push_back(target);
        }
    }

    //! Generate output messages
    RDX_INFO_DEV(this, __func__, "Got {} track targets, creating output messages", report_track_targets.size());
    using MsgTrackTarget = redoxi_public_msgs::msg::TrackTarget;
    std::vector<MsgTrackTarget> msg_track_targets;
    const auto &frame_metadata = goal->frame_bundle.primary_frame.metadata;
    for (const auto &target : filtered_track_targets) {
        MsgTrackTarget msg;
        auto bbox = target->get_bbox();
        msg.track_bbox.x = bbox.x;
        msg.track_bbox.y = bbox.y;
        msg.track_bbox.width = bbox.width;
        msg.track_bbox.height = bbox.height;
        msg.track_id = target->get_path_id();
        msg.track_status.bitmask = target->get_path_state();
        msg.confidence = target->get_confidence();
        msg.frame_metadata = frame_metadata;

        // do we have an associated detection?
        auto it_det = target2det.find(target);
        if (it_det != target2det.end()) {
            // yes, then find its original, and copy it to the message if exists
            auto it_orig_det = det2index.find(it_det->second);
            if (it_orig_det != det2index.end()) {
                auto index = it_orig_det->second;
                const auto &orig_det = goal->detections[index];
                msg.true_detection = orig_det;
                msg.true_detection.semantic_identity.value = msg.track_id;
                msg.true_detection.semantic_identity.confidence = msg.confidence;
            }
        }

        // create a predicted detection, with richer information
        // TODO: save all information of the tracked target here
        msg.predicted_detection = msg.true_detection; // inherit whatever is in true_detection first
        msg.predicted_detection.semantic_identity.value = msg.track_id;
        msg.predicted_detection.bbox = msg.track_bbox;
        // msg.predicted_detection.confidence = msg.confidence; // this is undefined
        msg.predicted_detection.semantic_identity.confidence = msg.confidence;

        // pend to sending list
        msg_track_targets.push_back(msg);
    }
    output_action_result->track_targets = msg_track_targets;

    RDX_INFO_DEV(this, __func__, "{}", "Input data handling completed");
    return 0;
}

int UniversalMotTrackerNode::_create_input_port_handler(
    const InitConfig_t &init_config,
    const RuntimeConfig_t &runtime_config)
{
    if (!m_input_port) {
        RDX_INFO_DEV(this, __func__, "{}", "No input port, skipping input port handler creation");
        return 0;
    }
    (void)init_config;

    auto handler_config = std::make_shared<InputPortHandler_t::InitConfig_t>();
    handler_config->block_input_reading = runtime_config.enable_blocking_mode;
    handler_config->block_resource_acquisition = runtime_config.enable_blocking_mode;

    // create input port handler
    m_input_port_handler = std::make_shared<InputPortHandler_t>();
    m_input_port_handler->init(m_input_port.get(), nullptr, handler_config);

    // add callback
    m_input_port_handler->on_process_input_data = std::bind(&UniversalMotTrackerNode::_handle_input_data,
                                                            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    return 0;
}

int UniversalMotTrackerNode::_create_tracker(const InitConfig_t &init_config)
{
    using Handler_t = RedoxiTrack::ExternalTrackingEventHandler;

    // create event handler
    auto event_handler = std::make_shared<Handler_t>();
    // event_handler->on_target_association_after =
    //     [this](Handler_t::TrackerBase_t *sender, const Handler_t::TargetAssociation_t &evt_data) {
    //         // TODO: record which detection is associated to which target
    //         return 0;
    //     };

    // event_handler->on_target_closed_after =
    //     [this](Handler_t::TrackerBase_t *sender, const Handler_t::TargetClosed_t &evt_data) {
    //         // TODO: record which target is closed
    //         return 0;
    //     };

    if (init_config.tracker_type == types::TrackerType::DeepSORT) {
        RDX_INFO_DEV(this, __func__, "{}", "Creating DeepSORT tracker");
        std::shared_ptr<DeepsortImpl> tracker_impl = std::make_shared<DeepsortImpl>();
        auto image_size = init_config.preferred_image_size;

        if (image_size.width > 0 && image_size.height > 0) {
            RDX_INFO_DEV(this, __func__, "Setting preferred image size to width={}, height={}", image_size.width, image_size.height);
            tracker_impl->m_tracker_param->set_preferred_image_size(image_size);
        }

        auto deepsort_params = std::static_pointer_cast<RedoxiTrack::DeepSortTrackerParam>(tracker_impl->m_tracker_param);
        deepsort_params->m_max_gating_distance = init_config.deep_sort_params.max_gating_distance;
        deepsort_params->m_base_gating_threshold = init_config.deep_sort_params.base_gating_threshold;
        deepsort_params->m_alpha_smooth_features = init_config.deep_sort_params.alpha_smooth_features;
        deepsort_params->m_gating_dist_lambda = init_config.deep_sort_params.gating_dist_lambda;
        deepsort_params->m_duplicate_iou_dist = init_config.deep_sort_params.duplicate_iou_dist;
        deepsort_params->m_use_optical_before_track = init_config.deep_sort_params.use_optical_before_track;

        RDX_INFO_DEV(this, __func__, "{}", "Initializing DeepSORT tracker");
        tracker_impl->m_tracker->init(*deepsort_params);
        tracker_impl->m_tracker_event_handler = event_handler;
        m_impl->m_tracker_impl = tracker_impl;
    } else if (init_config.tracker_type == types::TrackerType::BoTSort) {
        RDX_INFO_DEV(this, __func__, "{}", "Creating BoTSort tracker");
        std::shared_ptr<BotsortImpl> tracker_impl = std::make_shared<BotsortImpl>();
        auto image_size = init_config.preferred_image_size;
        if (image_size.width > 0 && image_size.height > 0) {
            RDX_INFO_DEV(this, __func__, "Setting preferred image size to width={}, height={}", image_size.width, image_size.height);
            tracker_impl->m_tracker_param->set_preferred_image_size(image_size);
        }

        auto botsort_params = std::static_pointer_cast<RedoxiTrack::BotsortTrackerParam>(tracker_impl->m_tracker_param);
        botsort_params->m_track_high_thresh = init_config.botsort_params.track_high_thresh;
        botsort_params->m_track_low_thresh = init_config.botsort_params.track_low_thresh;
        botsort_params->m_match_thresh = init_config.botsort_params.match_thresh;
        botsort_params->m_aspect_ratio_thresh = init_config.botsort_params.aspect_ratio_thresh;
        botsort_params->m_min_box_area = init_config.botsort_params.min_box_area;
        botsort_params->m_proximity_thresh = init_config.botsort_params.proximity_thresh;
        botsort_params->m_new_track_thresh = init_config.botsort_params.new_track_thresh;
        botsort_params->m_max_time_lost = init_config.botsort_params.max_time_lost;
        botsort_params->m_keep_track_buffer = init_config.botsort_params.keep_track_buffer;
        botsort_params->m_use_optical_before_track = init_config.botsort_params.use_optical_before_track;
        botsort_params->m_fuse_score = init_config.botsort_params.fuse_score;
        botsort_params->m_use_reid_feature = init_config.botsort_params.use_reid_feature;
        botsort_params->m_appearance_thresh = init_config.botsort_params.appearance_thresh;
        botsort_params->m_alpha_smooth_features = init_config.botsort_params.alpha_smooth_features;

        RDX_INFO_DEV(this, __func__, "{}", "Initializing BoTSort tracker");
        tracker_impl->m_tracker->init(*botsort_params);
        tracker_impl->m_tracker_event_handler = event_handler;
        m_impl->m_tracker_impl = tracker_impl;
    }
    m_impl->m_tracker_impl->m_tracker->add_event_handler(event_handler);
    return 0;
}

void UniversalMotTrackerNode::_step()
{
    if (m_input_port_handler) {
        m_input_port_handler->process_and_reply();
    }
}

int UniversalMotTrackerNode::_open()
{
    auto init_config = std::static_pointer_cast<InitConfig_t>(m_init_config);

    // recreate tracker on each open
    {
        RDX_INFO_DEV(this, __func__, "{}", "Creating tracker");
        auto ret = _create_tracker(*init_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("{}", "Failed to create tracker");
        }
    }
    return 0;
}

int UniversalMotTrackerNode::_start()
{
    // recreate input port handler on each start
    auto runtime_config = std::static_pointer_cast<RuntimeConfig_t>(m_runtime_config);
    auto init_config = std::static_pointer_cast<InitConfig_t>(m_init_config);
    {
        RDX_INFO_DEV(this, __func__, "{}", "Creating input port handler");
        auto ret = _create_input_port_handler(*init_config, *runtime_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("{}", "Failed to create input port handler");
        }
    }

    // start input port
    if (m_input_port) {
        m_input_port->start();
    }
    return 0;
}

int UniversalMotTrackerNode::_stop()
{
    // stop input port
    if (m_input_port) {
        m_input_port->stop();
    }
    return 0;
}

int UniversalMotTrackerNode::_close()
{
    // finish all tracking tasks
    if (m_impl->m_tracker_impl && m_impl->m_tracker_impl->m_tracker) {
        m_impl->m_tracker_impl->m_tracker->finish_track();
    }
    return 0;
}
} // namespace redoxi_works::model_nodes::universal_mot_trackers
