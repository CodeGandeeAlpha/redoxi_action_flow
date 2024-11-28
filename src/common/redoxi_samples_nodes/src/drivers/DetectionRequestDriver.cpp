#include <redoxi_samples_nodes/drivers/DetectionRequestDriver.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <typeinfo>

namespace redoxi_works::samples_nodes::drivers
{
DetectionRequestDriver::DetectionRequestDriver(const std::string &name, const rclcpp::NodeOptions &options)
    : common_nodes::StartStopNode(name, options)
{
}

int DetectionRequestDriver::_start()
{
    if (m_image_input_port) {
        m_image_input_port->start();
    }
    if (m_detection_request_output_port) {
        m_detection_request_output_port->start();
    }
    return 0;
}

int DetectionRequestDriver::_stop()
{
    //! Stop the node and cleanup resources
    if (m_image_input_port) {
        m_image_input_port->stop();
    }
    if (m_detection_request_output_port) {
        m_detection_request_output_port->stop();
    }
    return 0;
}

void DetectionRequestDriver::_step()
{
    //! Process incoming data and send detection requests
    if (m_image_request_port_handler) {
        m_image_request_port_handler->process_and_send();
    }
}

int DetectionRequestDriver::_update_init_config(std::shared_ptr<BaseInitConfig_t> init_config)
{
    //! Update initial configuration
    RDX_INFO_DEV(this, __func__, false, "{}", "Starting update of initial configuration");
    auto config = std::dynamic_pointer_cast<InitConfig_t>(init_config);
    if (!config) {
        RDX_RAISE_ERROR("Invalid init config, expected type: {}", typeid(InitConfig_t).name());
    }

    // create input port
    if (config->input_port_config && !config->input_port_config->get_action_name().empty()) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Creating input port with action name: {}", config->input_port_config->get_action_name());
        m_image_input_port = std::make_shared<ByImageRequest::InputPort_t>(this);
        m_image_input_port->init(config->input_port_config);
    }

    // create output port
    if (config->output_port_config) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Creating output port");
        m_detection_request_output_port = std::make_shared<OutputPort_t>(this);
        m_detection_request_output_port->init(config->output_port_config);
    }

    // create visualization publisher
    if (!config->publish_visualization_topic.empty()) {
        RDX_INFO_DEV(this, __func__, false, "{}", "Creating visualization publisher with topic: {}", config->publish_visualization_topic);
        m_pub_visualization = std::make_shared<StampedImagePub>();
        m_pub_visualization->init(this, config->publish_visualization_topic);
    }

    RDX_INFO_DEV(this, __func__, false, "{}", "Completed update of initial configuration");
    return 0;
}

int DetectionRequestDriver::_update_runtime_config(std::shared_ptr<BaseRuntimeConfig_t> runtime_config)
{
    RDX_INFO_DEV(this, __func__, false, "{}", "Updating runtime configuration");
    //! Update runtime configuration
    auto config = std::dynamic_pointer_cast<RuntimeConfig_t>(runtime_config);
    if (!config) {
        RDX_RAISE_ERROR("Invalid runtime config, expected type: {}", typeid(RuntimeConfig_t).name());
    }

    // create port handler
    if (m_image_input_port) {
        auto handler_config = std::make_shared<port_handlers::PullProcessSendHandlerConfig>();
        handler_config->block_input_reading = config->enable_blocking_mode;
        handler_config->block_resource_acquisition = config->enable_blocking_mode;
        m_image_request_port_handler = std::make_shared<ImageRequestPortHandler_t>();

        RDX_INFO_DEV(this, __func__, false, "{}", "Initializing port handler");
        m_image_request_port_handler->init(
            m_image_input_port.get(),
            m_detection_request_output_port.get(),
            nullptr,
            handler_config,
            config->output_enqueue_policy,
            this);

        m_image_request_port_handler->on_process_input_data =
            [this, config](OutputRequest_t *output_request,
                           ByImageRequest::ActionResult_t *output_result,
                           std::shared_ptr<ByImageRequest::SourceData_t> source_data,
                           auto &resource_token) {
                (void)resource_token;
                (void)output_result;

                // create request
                RDX_INFO_DEV(this, __func__, false, "{}", "Creating request from source data");
                OutputSourceData_t output_source_data;
                cv::Mat image;
                if (_extract_image(&image, *source_data)) {
                    output_source_data.set_image(image);
                }
                output_source_data.set_frame_metadata(source_data->get_goal()->frame.metadata);
                output_request->set_source_data(output_source_data);

                // publish visualization
                bool do_publish_visualization = config->enable_visualization && m_pub_visualization && !image.empty();
                if (do_publish_visualization) {
                    RDX_INFO_DEV(this, __func__, false, "{}", "Publishing visualization");
                    m_pub_visualization->publish(image);
                }

                return 0;
            };
    }

    return 0;
}

int DetectionRequestDriver::_extract_image(cv::Mat *output_image, const ByImageRequest::SourceData_t &source_data)
{
    if (!output_image) {
        return 0;
    }

    const auto &raw_image = source_data.get_goal()->frame.raw_image;
    if (!raw_image.data.empty()) {
        *output_image = cv_bridge::toCvCopy(raw_image, raw_image.encoding)->image;
    }
    return 0;
}

} // namespace redoxi_works::samples_nodes::drivers