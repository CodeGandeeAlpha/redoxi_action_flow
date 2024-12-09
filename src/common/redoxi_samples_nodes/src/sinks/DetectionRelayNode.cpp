#include <redoxi_samples_nodes/sinks/DetectionRelayNode.hpp>
#include <redoxi_dnn_models/message_conversion.hpp>
#include <redoxi_dnn_models/visualizations.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <redoxi_common_cpp/image_proc/FrameMediator.hpp>
namespace redoxi_works
{

DetectionRelayNode::DetectionRelayNode(const std::string &node_name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(node_name, options)
{
}

int DetectionRelayNode::_update_init_config(std::shared_ptr<BaseInitConfig_t> config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "updating init config");

    // type check
    auto init_config = std::dynamic_pointer_cast<InitConfig_t>(config);
    if (!init_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast init config to DetectionRelayNodeInitConfig", __func__);
    }

    // create input port
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating input port");
    m_input_port = std::make_shared<InputPort_t>(this);
    m_input_port->init(init_config->input_port_config);

    // create publishers
    RDX_INFO_DEV(this, __func__, false, "{}", "Initializing publishers");
    if (init_config->publish_visualization_topic.empty()) {
        RDX_INFO_DEV(this, __func__, false, "{}", "visualization publishing is disabled");
    } else {
        m_pub_visualization.init(this, init_config->publish_visualization_topic, StampedImagePub::DefaultQoS);
    }

    RDX_INFO_DEV(this, __func__, false, "{}", "Init config update completed successfully");
    return 0;
}

int DetectionRelayNode::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "updating runtime config");
    auto runtime_config = std::dynamic_pointer_cast<RuntimeConfig_t>(config);
    if (!runtime_config) {
        RDX_RAISE_ERROR("[{}] Failed to cast runtime config to DetectionRelayNodeRuntimeConfig", __func__);
    }

    // create port handler
    auto ret = _create_port_handler(runtime_config);
    if (ret != 0) {
        RDX_RAISE_ERROR("[{}] Failed to create port handler, ret = {}", __func__, ret);
    }

    return 0;
}

int DetectionRelayNode::_start()
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting detection relay node");

    if (m_input_port) {
        m_input_port->start();
        RDX_INFO_DEV(this, __func__, false, "{}", "input port started");
    }

    // step thread and state will be handled by base class
    return 0;
}

int DetectionRelayNode::_stop()
{
    m_input_port->stop();
    RDX_INFO_DEV(this, __func__, false, "{}", "input port stopped");

    return 0;
}

void DetectionRelayNode::_step()
{
    if (m_port_handler) {
        m_port_handler->process_and_reply();
    }
}

int DetectionRelayNode::_create_port_handler(
    std::shared_ptr<RuntimeConfig_t> runtime_config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Creating port handler");
    m_port_handler = std::make_shared<PortHandler_t>();
    auto port_handler_config = std::make_shared<PortHandler_t::InitConfig_t>();
    port_handler_config->block_input_reading = runtime_config->enable_blocking_mode;
    port_handler_config->block_resource_acquisition = runtime_config->enable_blocking_mode;
    m_port_handler->init(m_input_port.get(), nullptr, port_handler_config, this);

    m_port_handler->on_process_input_data =
        [this, runtime_config](InputPort_t::ActionResult_t *output_action_result,
                               std::shared_ptr<SourceData_t> source_data,
                               auto &resource_token) {
            RDX_INFO_DEV(this, __func__, false, "{}", "processing input data");
            (void)resource_token;
            (void)output_action_result;

            image_utils::FrameMediator fm(&source_data->get_goal()->frame_bundle.primary_frame);
            auto encoding = fm.get_encoding();

            // publish visualization
            if (runtime_config->enable_visualization && m_pub_visualization.valid()) {
                auto msg_uuid = ActionDataTrait_t::get_uuid(*source_data->get_goal());
                RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Publishing visualization",
                             boost::uuids::to_string(msg_uuid));
                cv::Mat vis_image;
                auto ret = _parse_detection(&vis_image, *source_data);
                if (ret != 0) {
                    RDX_INFO_DEV(this, __func__, false, "Failed to parse detection, ret = {}", ret);
                    return ret;
                } else if (vis_image.empty()) {
                    RDX_INFO_DEV(this, __func__, false, "{}", "Visualization image is empty, skipping");
                } else {
                    RDX_INFO_DEV(this, __func__, false, "Visualization image size: {}x{}", vis_image.cols, vis_image.rows);
                    m_pub_visualization.publish(vis_image, encoding);
                }
            }

            return 0;
        };
    // nothing to do here
    return 0;
}

int DetectionRelayNode::_parse_detection(cv::Mat *output,
                                         const SourceData_t &source_data)
{
    if (output == nullptr) {
        return 0;
    }

    auto msg_uuid = ActionDataTrait_t::get_uuid(*source_data.get_goal());

    // Assuming the detection data is directly available in source_data
    RDX_INFO_DEV(this, __func__, false, "[msg_uuid={}] Parsing detection data",
                 boost::uuids::to_string(msg_uuid));

    try {
        image_utils::FrameMediator fm(&source_data.get_goal()->frame_bundle.primary_frame);
        fm.to_cv_image_copy(*output);
    } catch (cv_bridge::Exception &e) {
        RDX_INFO_DEV(this, __func__, false, "cv_bridge exception: {}, ignoring the error", e.what());
    }

    image_utils::DrawDetectionsOptions options;
    options.colorization_mode = image_utils::DrawDetectionsOptions::ColorizationMode::ClassId;
    image_utils::draw_detections(output,
                                 source_data.get_goal()->detections,
                                 options);

    return 0;
}


} // namespace redoxi_works