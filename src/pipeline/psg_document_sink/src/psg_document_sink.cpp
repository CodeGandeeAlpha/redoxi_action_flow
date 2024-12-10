#include <psg_document_sink/psg_document_sink.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>


namespace redoxi_works
{

PSGDocumentSinkInitConfig::PSGDocumentSinkInitConfig()
{
    input_port_config->set_action_name("in/action");
    input_port_config->set_buffer_capacity(1);
}

PSGDocumentSink::PSGDocumentSink(const std::string &node_name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(node_name, options)
{
}

PSGDocumentSink::~PSGDocumentSink()
{
}

int PSGDocumentSink::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "updating init config");

    //! Step 1: Cast config to InitConfig_t
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(config);
    if (!init_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast init config to FrameRelayNodeInitConfig", __func__);
    }

    //! Step 2: Log init config as JSON for debugging
    // {
    //     RDX_INFO_DEV(this, __func__, false, "{}", "Parsing init config as JSON");
    //     auto init_config_json = JS::serializeStruct(*init_config);
    //     RDX_INFO_DEV(this, __func__, false, "InitConfig JSON: {}", init_config_json);
    // }

    //! Step 3: Create and initialize input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating input port");
    m_input_port = std::make_shared<InputPort_t>(this);
    m_input_port->init(init_config->input_port_config);

    //! Step 4: Initialize publishers
    RDX_INFO_DEV(this, __func__, false, "{}", "Initializing publishers");
    m_pub_relayed_document.init(this, init_config->publish_topic, StampedImagePub::DefaultQoS);
    m_pub_relayed_image.init(this, init_config->publish_topic_image, StampedImagePub::DefaultQoS);
    m_pub_debug_document_accepted.init(this, init_config->debug_topic_document_accepted, StampedImagePub::DefaultUnreliableQoS);
    m_pub_debug_document_rejected.init(this, init_config->debug_topic_document_rejected, StampedImagePub::DefaultUnreliableQoS);

    RDX_INFO_DEV(this, __func__, false, "{}", "Init config update completed successfully");
    return 0;
}

int PSGDocumentSink::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "updating runtime config");
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(config);
    if (!runtime_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast runtime config to FrameRelayNodeRuntimeConfig", __func__);
    }

    // {
    //     RDX_INFO_DEV(this, __func__, false, "{}", "Converting runtime config to JSON");
    //     auto runtime_config_json = JS::serializeStruct(*runtime_config);
    //     RDX_INFO_DEV(this, __func__, false, "RuntimeConfig JSON: {}", runtime_config_json);
    // }

    return 0;
}


int PSGDocumentSink::_start()
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting frame relay node");

    m_input_port->start();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port started");

    return 0;
}

int PSGDocumentSink::_stop()
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Stopping frame relay node");

    m_input_port->stop();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port stopped");

    return 0;
}

void PSGDocumentSink::_step()
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(get_runtime_config());
    // get data from input port
    // RDX_INFO_DEV(this, __func__, false, "{}", "getting data from input port");
    std::shared_ptr<SourceData_t> psg_document_data;
    if (runtime_config->enable_blocking_mode) {
        // wait until there is data available
        psg_document_data = m_input_port->pop_source_data();
    } else {
        // try to get data without waiting
        psg_document_data = m_input_port->try_pop_source_data();
    }

    // no frame data found
    if (!psg_document_data) {
        // RDX_INFO_DEV(this, __func__, false, "{}", "no frame data found");
        return;
    }

    // get goal handle
    RDX_INFO_DEV(this, __func__, false, "{}", "frame received, getting goal handle");
    auto goal_handle = psg_document_data->get_goal_handle_future().get();
    auto goal_uuid = to_boost_uuid(psg_document_data->get_goal_uuid());
    if (!goal_handle) {
        RDX_INFO_DEV(this, __func__, false, "[goal_uuid={}] {}",
                     boost::uuids::to_string(goal_uuid), "no goal handle found");
        return;
    }

    // get document and publish
    const auto msg_uuid = ActionDataTrait_t::get_uuid(*goal_handle->get_goal());
    const auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] {}",
                 msg_uuid_str, "psg document received and goal resolved");
    auto &raw_document = goal_handle->get_goal()->document;
    m_pub_relayed_document.publish(raw_document);


    // publish image
    image_utils::FrameMediator fm(&raw_document.frame_bundle.primary_frame);
    sensor_msgs::msg::Image raw_image;
    fm.to_image_msg(raw_image);

    // publish debug topic?
    if (runtime_config->enable_debug_topics) {
        m_pub_debug_document_accepted.publish(raw_image, "accepted");
        RDX_INFO_DEV(this, __func__, false, "{}", "published debug document accepted");
    }

    // at the end, terminate the goal
    auto result_msg = std::make_shared<InputPort_t::ActionResult_t>();
    goal_handle->abort(result_msg);
}

} // namespace redoxi_works
