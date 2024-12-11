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
        m_pub_probe = this->create_publisher<std_msgs::msg::String>(init_config->publish_probe_topic, DefaultParams::ProbePublisherQoS);
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
    RDX_INFO_DEV(this, __func__, "{}", "Handling input data");
    (void)resource_token;

    //! Check goal validity
    const auto *goal = source_data->get_goal();
    if (goal == nullptr) {
        RDX_RAISE_ERROR("{}", "Goal is not found, this is unexpected");
    }

    //! Extract input image
    cv::Mat image;
    {
        RDX_INFO_DEV(this, __func__, "{}", "Extracting image");
        // auto ret = _extract_image(&image, source_data.get());
        image_utils::FrameMediator fm(&source_data->get_goal()->frame_bundle.primary_frame);
        auto ret = fm.to_cv_image_copy(image);

        // no image? exit
        if (ret != 0) {
            return -1;
        }
    }

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

    //! Execute tracking
    RDX_INFO_DEV(this, __func__, "{}", "Executing tracking");
    auto tracker = m_impl->m_tracker_impl->m_tracker;
    auto tracking_frame_number = tracker->get_current_frame_number();
    auto current_frame_number = goal->frame_bundle.primary_frame.metadata.frame_num;
    if (tracking_frame_number == rxt::INIT_TRACKING_FRAME) {
        // init tracking
        tracker->begin_track(image, track_detections, current_frame_number);
    } else {
        // update tracking
        tracker->track(image, track_detections, current_frame_number);
    }

    //! Handle end of tracking if needed
    auto signal_code = InputActionDataTrait_t::get_control_signal_code(*goal);
    bool end_of_tracking = signal_code == ControlSignalCode::Flush || signal_code == ControlSignalCode::Terminate;
    if (end_of_tracking) {
        RDX_INFO_DEV(this, __func__, "{}", "Finishing tracking");
        tracker->finish_track();
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

    //! Generate output messages
    RDX_INFO_DEV(this, __func__, "Got {} track targets, creating output messages", report_track_targets.size());
    using MsgTrackTarget = redoxi_public_msgs::msg::TrackTarget;
    std::vector<MsgTrackTarget> msg_track_targets;
    const auto &frame_metadata = goal->frame_bundle.primary_frame.metadata;
    for (const auto &target : report_track_targets) {
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
            }
        }

        // create a predicted detection, with richer information
        // TODO: save all information of the tracked target here
        msg.predicted_detection = msg.true_detection; // inherit whatever is in true_detection first
        msg.predicted_detection.category = msg.track_id;
        msg.predicted_detection.bbox = msg.track_bbox;
        msg.predicted_detection.confidence = msg.confidence;

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
    m_input_port_handler->init(m_input_port.get(), nullptr, handler_config, this);

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

        RDX_INFO_DEV(this, __func__, "{}", "Initializing DeepSORT tracker");
        tracker_impl->m_tracker->init(*tracker_impl->m_tracker_param);
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

        RDX_INFO_DEV(this, __func__, "{}", "Initializing BoTSort tracker");
        tracker_impl->m_tracker->init(*tracker_impl->m_tracker_param);
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
