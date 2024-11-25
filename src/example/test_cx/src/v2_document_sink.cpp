#include <psg_document_sink/AsyncDocumentInputPort.hpp>
#include <psg_document_sink/psg_document_sink.hpp>

namespace rdx = redoxi_works;

void print_init_config_json()
{
    auto init_config = std::make_shared<rdx::PSGDocumentSinkInitConfig>();
    auto js_string = JS::serializeStruct(*init_config);
    std::cout << js_string << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //! Create a node options object
    rclcpp::NodeOptions options;

    //! Create a FrameRelayNode with a specific name and options
    auto document_sink_node = std::make_shared<rdx::PSGDocumentSink>("document_sink_node", options);

    //! Initialize the node with default configuration
    auto init_config = std::make_shared<rdx::PSGDocumentSinkInitConfig>();
    init_config->parse_from_node_parameters(init_config.get(), document_sink_node.get());
    {
        RDX_INFO_DEV(document_sink_node.get(), __func__, false, "{}", "Converting init config to JSON");
        auto init_config_json = JS::serializeStruct(*init_config);
        RDX_INFO_DEV(document_sink_node.get(), __func__, false, "InitConfig JSON: {}", init_config_json);
    }
    auto runtime_config = std::make_shared<rdx::PSGDocumentSinkRuntimeConfig>();
    runtime_config->parse_from_node_parameters(runtime_config.get(), document_sink_node.get());
    {
        RDX_INFO_DEV(document_sink_node.get(), __func__, false, "{}", "Converting runtime config to JSON");
        auto runtime_config_json = JS::serializeStruct(*runtime_config);
        RDX_INFO_DEV(document_sink_node.get(), __func__, false, "RuntimeConfig JSON: {}", runtime_config_json);
    }
    document_sink_node->init(init_config, runtime_config);

    //! Start the node
    document_sink_node->start();

    //! Keep the node running
    rclcpp::spin(document_sink_node);

    //! Stop the node before shutdown
    document_sink_node->stop();

    //! Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
