#include <redoxi_samples_nodes/sinks/FrameRelayNode.hpp>
#include <boost/uuid/uuid_io.hpp>


namespace redoxi_works
{

FrameRelayNodeInitConfig::FrameRelayNodeInitConfig()
{
    {
        auto p = input_port_config;
        p->set_action_name("in/action");
        p->set_buffer_capacity(1);
    }
}

void FrameRelayNodeInitConfig::from_parameters(const FrameRelayNode *node)
{
    auto &json_params = node->get_json_parameters();

    //! Retrieve the init_config from the top level JSON parameters
    if (json_params.contains("init_config")) {
        //! Convert the init_config to a string
        std::string init_config_str = json_params["init_config"].dump();

        //! Parse the string into the InitConfig structure using json_struct
        JS::ParseContext pc(init_config_str);
        auto ret = pc.parseTo(*this);
        if (ret != JS::Error::NoError) {
            RDX_RAISE_ERROR("[{}] Failed to parse init_config from JSON parameters: {}",
                            __func__, JS::Internal::error_strings[static_cast<int>(ret)]);
        }
        RDX_INFO_DEV(node, __func__, false, "{}", "init_config parsed from JSON parameters");
    } else {
        RDX_INFO_DEV(node, __func__, false, "{}", "init_config not found in JSON parameters, using default values");
    }
}

FrameRelayNode::FrameRelayNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options)
{
    auto ret = declare_default_parameters_for_node(this);
    if (ret != 0) {
        RDX_RAISE_ERROR("[{}] Failed to declare default parameters for node", __func__);
    }

    auto node = this;
    m_json_parameters = RDX_GET_JSON_PARAM_FROM_NODE(node);
}

FrameRelayNode::~FrameRelayNode()
{
    stop();
}

int FrameRelayNode::init(std::shared_ptr<InitConfig_t> init_config)
{
    m_init_config = init_config;

    {
        //! Convert the InitConfig structure to a JSON string for debugging
        std::string init_config_json = JS::serializeStruct(*m_init_config);
        RDX_INFO_DEV(this, __func__, false, "{}", "InitConfig JSON: {}", init_config_json);
    }

    // create the input port
    m_input_port = std::make_shared<InputPort_t>(this);
    m_input_port->init(m_init_config->input_port_config);

    // create the publishers
    m_pub_relayed_frame.init(this, m_init_config->publish_topic, StampedImagePub::DefaultQoS);
    m_pub_frame_accepted.init(this, m_init_config->debug_topic_frame_accepted, StampedImagePub::DefaultUnreliableQoS);
    m_pub_frame_rejected.init(this, m_init_config->debug_topic_frame_rejected, StampedImagePub::DefaultUnreliableQoS);

    // enable debug topics?
    set_debug_topics_enabled(m_init_config->enable_debug_topics);

    return 0;
}

int FrameRelayNode::start()
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting frame relay node");

    m_input_port->start();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port started");

    m_running = true;
    auto step_interval = m_init_config->step_interval;
    m_step_thread = std::make_shared<std::thread>([this, step_interval] {
        while (this->m_running && rclcpp::ok()) {
            auto start_time = std::chrono::high_resolution_clock::now();
            this->_step();
            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed_time = end_time - start_time;
            auto sleep_time = step_interval - elapsed_time;
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
    });

    RDX_INFO_DEV(this, __func__, false, "{}", "step thread started");
    return 0;
}

int FrameRelayNode::stop()
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Stopping frame relay node");

    m_input_port->stop();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port stopped");

    m_running = false;
    if (m_step_thread) {
        m_step_thread->join();
    }
    RDX_INFO_DEV(this, __func__, false, "{}", "step thread stopped");

    return 0;
}

void FrameRelayNode::_step()
{
    // get data from input port
    // RDX_INFO_DEV(this, __func__, false, "{}", "getting data from input port");
    std::shared_ptr<SourceData_t> frame_data;
    if (m_init_config->enable_blocking_mode) {
        // wait until there is data available
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

    // get frame and publish
    const auto msg_uuid = ActionDataTrait_t::get_uuid(*goal_handle->get_goal());
    const auto msg_uuid_str = boost::uuids::to_string(msg_uuid);
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] {}",
                 msg_uuid_str, "frame received and goal resolved");
    auto &raw_image = goal_handle->get_goal()->frame.raw_image;
    m_pub_relayed_frame.publish(raw_image);

    // publish debug topic?
    if (get_debug_topics_enabled()) {
        m_pub_frame_accepted.publish(raw_image, "accepted");
    }

    // at the end, terminate the goal
    auto result_msg = std::make_shared<InputPort_t::ActionResult_t>();
    goal_handle->abort(result_msg);
}

} // namespace redoxi_works
