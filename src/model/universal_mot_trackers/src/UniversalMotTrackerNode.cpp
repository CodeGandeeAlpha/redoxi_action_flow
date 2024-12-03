#include <universal_mot_trackers/UniversalMotTrackerNode.hpp>
#include <universal_mot_trackers/UniversalMotTrackerImpl.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

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

    // create tracker
    {
        RDX_INFO_DEV(this, __func__, "{}", "Creating tracker");
        auto ret = _create_tracker(*init_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("{}", "Failed to create tracker");
        }
    }

    // create port
    {
        RDX_INFO_DEV(this, __func__, "{}", "Creating port");
        m_input_port = std::make_shared<InputPort_t>(this);
        auto ret = m_input_port->init(init_config->input_port_config);
        if (ret != 0) {
            RDX_RAISE_ERROR("{}", "Failed to create port");
        }
    }

    // create publisher for visualization
    if (!init_config->publish_visualization_topic.empty()) {
        RDX_INFO_DEV(this, __func__, "{}", "Creating publisher for visualization");
        m_pub_visualization = std::make_shared<StampedImagePub>(this, init_config->publish_visualization_topic);
    }

    return 0;
}

int UniversalMotTrackerNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
{
    auto runtime_config = std::static_pointer_cast<RuntimeConfig_t>(config);

    return 0;
}

int UniversalMotTrackerNode::_create_tracker(const InitConfig_t &init_config)
{
    using Handler_t = RedoxiTrack::ExternalTrackingEventHandler;

    // create event handler
    auto event_handler = std::make_shared<Handler_t>();
    event_handler->on_target_association_after =
        [this](Handler_t::TrackerBase_t *sender, const Handler_t::TargetAssociation_t &evt_data) {
            // TODO: record which detection is associated to which target
            return 0;
        };

    event_handler->on_target_closed_after =
        [this](Handler_t::TrackerBase_t *sender, const Handler_t::TargetClosed_t &evt_data) {
            // TODO: record which target is closed
            return 0;
        };

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
    return 0;
}

} // namespace redoxi_works::model_nodes::universal_mot_trackers
