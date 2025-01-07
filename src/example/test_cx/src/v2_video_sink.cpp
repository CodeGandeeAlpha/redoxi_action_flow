#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>
#include <redoxi_samples_nodes/sinks/FrameRelayNode.hpp>

namespace rdx = redoxi_works;

void print_init_config_json()
{
    auto init_config = std::make_shared<rdx::FrameRelayNodeInitConfig>();
    auto js_string = JS::serializeStruct(*init_config);
    std::cout << js_string << std::endl;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //! Create a node options object
    rclcpp::NodeOptions options;

    //! Create a FrameRelayNode with a specific name and options
    auto frame_relay_node = std::make_shared<rdx::FrameRelayNode>("frame_relay_node", options);

    //! Initialize the node with default configuration
    auto init_config = std::make_shared<rdx::FrameRelayNodeInitConfig>();
    init_config->from_parameters(frame_relay_node.get());
    frame_relay_node->init(init_config);

    //! Start the node
    frame_relay_node->start();

    //! Keep the node running
    rclcpp::spin(frame_relay_node->get_node_base_interface());

    //! Stop the node before shutdown
    frame_relay_node->stop();

    //! Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
