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
    init_config->from_parameters(document_sink_node.get());
    document_sink_node->init(init_config);

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
