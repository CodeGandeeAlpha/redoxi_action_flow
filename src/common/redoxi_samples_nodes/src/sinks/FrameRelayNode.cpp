#include <redoxi_samples_nodes/sinks/FrameRelayNode.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace redoxi_works
{

FrameRelayNodeInitConfig::FrameRelayNodeInitConfig()
{
    // default configuration for input port
    input_port_config->set_action_name("in/action");
    input_port_config->set_buffer_capacity(1);
}

FrameRelayNode::FrameRelayNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(node_name, options)
{
}

int FrameRelayNode::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
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
    m_pub_relayed_frame.init(this, init_config->publish_topic, RelayedFrameQoS);
    m_pub_frame_accepted.init(this, init_config->debug_topic_frame_accepted, StampedImagePub::DefaultUnreliableQoS);
    m_pub_frame_rejected.init(this, init_config->debug_topic_frame_rejected, StampedImagePub::DefaultUnreliableQoS);

    RDX_INFO_DEV(this, __func__, false, "{}", "Init config update completed successfully");
    return 0;
}

int FrameRelayNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
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

    // enable debug topics?
    set_debug_topics_enabled(runtime_config->enable_debug_topics);

    return 0;
}

int FrameRelayNode::_start()
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting frame relay node");

    if (m_input_port) {
        m_input_port->start();
        RDX_INFO_DEV(this, __func__, false, "{}", "input port started");
    }

    // create shm client
    {
        m_shm_client = shared_memory::SharedMemoryFactory::get_instance().get_default_client(this);
        auto shm_client = m_shm_client.lock();
        const auto &config = shm_client->get_shm_config();
        if (!shm_client) {
            RDX_INFO_DEV(this, __func__, false, "{}", "Failed to create shm client");
        } else {
            RDX_INFO_DEV(this, __func__, false, "Created shm client, service type = {}, region key = {}",
                         config.service_type, config.region_key);
        }
    }

    // step thread and state will be handled by base class
    return 0;
}

int FrameRelayNode::_stop()
{
    m_input_port->stop();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port stopped");

    // delete shm client, disconnect from shm service
    m_shm_client.reset();

    return 0;
}

void FrameRelayNode::_step()
{
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(get_runtime_config());
    if (!runtime_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast runtime config to FrameRelayNodeRuntimeConfig", __func__);
    }

    // get data from input port
    // RDX_INFO_DEV(this, __func__, false, "{}", "getting data from input port");
    std::shared_ptr<SourceData_t> frame_data;
    if (runtime_config->enable_blocking_mode) {
        // wait until there is data available
        RDX_INFO_DEV(this, __func__, false, "{}", "Reading in blocking mode, waiting for data from input port");
        frame_data = m_input_port->pop_source_data();
    } else {
        // try to get data without waiting
        frame_data = m_input_port->try_pop_source_data();
    }

    // no frame data found
    if (!frame_data) {
        // RDX_INFO_DEV(this, __func__, false, "{}", "no frame data found");
        return;
    }

    // get goal handle
    RDX_INFO_DEV(this, __func__, false, "{}", "frame received, getting goal handle");
    auto goal_handle = frame_data->get_goal_handle_future().get();
    auto goal_uuid = to_boost_uuid(frame_data->get_goal_uuid());
    if (!goal_handle) {
        RDX_INFO_DEV(this, __func__, false, "[goal_uuid={}] {}",
                     boost::uuids::to_string(goal_uuid), "no goal handle found");
        return;
    }

    // flush signal?
    auto control_signal_code = ActionDataTrait_t::get_control_signal_code(*goal_handle->get_goal());
    if (control_signal_code == ControlSignalCode::Flush) {
        RDX_INFO_DEV(this, __func__, false, "[goal_uuid={}] {}",
                     boost::uuids::to_string(goal_uuid), "flush signal received");
    }

    // get frame and publish
    const auto msg_uuid = ActionDataTrait_t::get_uuid(*goal_handle->get_goal());
    const auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] {}",
                 msg_uuid_str, "frame received and goal resolved");

    cv::Mat frame;
    image_utils::FrameMediator fm(&frame_data->get_goal()->frame_bundle.primary_frame);
    auto frame_number = fm.get_frame_number();
    auto task_uuid = InputPort_t::ActionDataTrait_t::get_source_task_metadata(*frame_data->get_goal()).source_task_id;
    auto got_frame = fm.to_cv_image_copy(frame) == 0;
    if (got_frame) {
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}][task={}] got frame, frame number={}, signal code={}",
                     msg_uuid_str, UUIDTrait::to_string(task_uuid), frame_number, control_signal_code_to_string(control_signal_code));

        // _parse_frame(&frame, *frame_data);
        auto encoding = frame_data->get_goal()->frame_bundle.primary_frame.metadata.encoding;
        m_pub_relayed_frame.publish(frame, encoding);

        // publish debug topic?
        if (get_debug_topics_enabled()) {
            auto control_signal_code = ActionDataTrait_t::get_control_signal_code(*frame_data->get_goal());
            auto label_text = fmt::format("accepted, signal = {}", control_signal_code_to_string(control_signal_code));
            m_pub_frame_accepted.publish(frame, encoding, label_text);
        }
    } else {
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}][task={}] failed to get frame, frame number={}, signal code={}",
                     msg_uuid_str, UUIDTrait::to_string(task_uuid), frame_number, control_signal_code_to_string(control_signal_code));
    }

    // at the end, terminate the goal
    auto result_msg = std::make_shared<InputPort_t::ActionResult_t>();
    goal_handle->succeed(result_msg);
}

int FrameRelayNode::_parse_frame(cv::Mat *output,
                                 const SourceData_t &source_data)
{
    if (output == nullptr) {
        return 0;
    }

    auto msg_uuid = ActionDataTrait_t::get_uuid(*source_data.get_goal());

    // parse from shm
    auto &shm_token = source_data.get_goal()->frame_bundle.primary_frame.shm_token;
    auto shm_client = m_shm_client.lock();
    if (shm_token.object_size >= 0 && shm_client && shm_client->is_connected()) {
        shared_memory::ObjectIdentifier oid;
        if (shm_token.object_id != 0) {
            oid.id = shm_token.object_id;
        }
        if (!shm_token.object_key.empty()) {
            oid.key = shm_token.object_key;
        }
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Getting data from shm with object id {}",
                     boost::uuids::to_string(msg_uuid), oid.id.value_or(0));
        auto datablock = shm_client->get_data(oid);
        if (!datablock) {
            RDX_INFO_DEV(this, __func__, false,
                         "[msg_uuid={}] Failed to get data from shm", boost::uuids::to_string(msg_uuid));
            return -1;
        }

        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Data from shm parsed to cv::Mat",
                     boost::uuids::to_string(msg_uuid));
        cv::Mat tmp;
        datablock->get_as_cvmat(&tmp);

        // IMPORTANT: copy the data to the output, because the datablock will be released after the function returns
        tmp.copyTo(*output);

        // after reading, delete the data from shm
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Deleting data from shm",
                     boost::uuids::to_string(msg_uuid));
        auto delete_ok = shm_client->delete_object(oid) == 0;
        if (!delete_ok) {
            RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Failed to delete data from shm",
                         boost::uuids::to_string(msg_uuid));
        }
    } else {
        // read raw image directly from the goal
        RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Reading raw image directly from the goal",
                     boost::uuids::to_string(msg_uuid));

        image_utils::FrameMediator fm(&source_data.get_goal()->frame_bundle.primary_frame);
        fm.to_cv_image_copy(*output);
        // auto &raw_image = source_data.get_goal()->frame_bundle.primary_frame.raw_image;
        // if (!raw_image.data.empty()) {
        //     auto img_bridge = cv_bridge::toCvCopy(raw_image, raw_image.encoding);
        //     *output = img_bridge->image;
        // }
    }

    return 0;
}

} // namespace redoxi_works
