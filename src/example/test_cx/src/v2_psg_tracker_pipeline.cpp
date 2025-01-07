#include <psg_tracker/Pipeline.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //! Create a node options object
    rclcpp::NodeOptions options;

    //! Create a FrameRelayNode with a specific name and options
    auto psg_tracker_pipeline_node = std::make_shared<rdx::PSGTrackerPipelineNode>("psg_tracker_pipeline_node", options);

    //! Initialize the node with default configuration
    auto init_config = std::make_shared<rdx::PSGTrackerPipelineNode::InitConfig_t>();
    init_config->parse_from_node_parameters(init_config.get(), psg_tracker_pipeline_node.get());
    {
        RDX_INFO_DEV(psg_tracker_pipeline_node.get(), __func__, false, "{}", "Converting init config to JSON");
        auto init_config_json = JS::serializeStruct(*init_config);
        RDX_INFO_DEV(psg_tracker_pipeline_node.get(), __func__, false, "InitConfig JSON: {}", init_config_json);
    }
    auto runtime_config = std::make_shared<rdx::PSGTrackerPipelineNode::RuntimeConfig_t>();
    runtime_config->parse_from_node_parameters(runtime_config.get(), psg_tracker_pipeline_node.get());
    {
        RDX_INFO_DEV(psg_tracker_pipeline_node.get(), __func__, false, "{}", "Converting runtime config to JSON");
        auto runtime_config_json = JS::serializeStruct(*runtime_config);
        RDX_INFO_DEV(psg_tracker_pipeline_node.get(), __func__, false, "RuntimeConfig JSON: {}", runtime_config_json);
    }

    psg_tracker_pipeline_node->init(init_config, runtime_config);

    //! Start the node
    psg_tracker_pipeline_node->start();

    //! Keep the node running
    rclcpp::spin(psg_tracker_pipeline_node->get_node_base_interface());

    //! Stop the node before shutdown
    psg_tracker_pipeline_node->stop();

    //! Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
