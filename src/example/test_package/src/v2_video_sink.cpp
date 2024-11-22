#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>
#include <redoxi_samples_nodes/sinks/FrameRelayNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace rdx = redoxi_works;

void print_init_config_json()
{
    {
        auto init_config = std::make_shared<rdx::FrameRelayNodeInitConfig>();
        auto js_string = JS::serializeStruct(*init_config);
        std::cout << js_string << std::endl;
    }
    {
        auto runtime_config = std::make_shared<rdx::FrameRelayNodeRuntimeConfig>();
        auto js_string = JS::serializeStruct(*runtime_config);
        std::cout << js_string << std::endl;
    }
}

int main(int argc, char **argv)
{
    // print_init_config_json();
    // return 0;
    rclcpp::init(argc, argv);

    //! Create a node options object
    rclcpp::NodeOptions options;

    //! Create a FrameRelayNode with a specific name and options
    // RDX_INFO_DEV(nullptr, __func__, false, "{}", "Creating frame relay node");
    auto frame_relay_node = std::make_shared<rdx::FrameRelayNode>("frame_relay_node", options);

    //! Initialize the node with default configuration
    RDX_INFO_DEV(frame_relay_node.get(), __func__, false, "{}", "Creating init config");
    auto init_config = std::make_shared<rdx::FrameRelayNodeInitConfig>();

    RDX_INFO_DEV(frame_relay_node.get(), __func__, false, "{}", "Creating runtime config");
    auto runtime_config = std::make_shared<rdx::FrameRelayNodeRuntimeConfig>();

    RDX_INFO_DEV(frame_relay_node.get(), __func__, false, "{}", "Parsing init config from node parameters");
    init_config->parse_from_node_parameters(init_config.get(), frame_relay_node.get());
    {
        RDX_INFO_DEV(frame_relay_node.get(), __func__, false, "{}", "Converting init config to JSON");
        auto init_config_json = JS::serializeStruct(*init_config);
        RDX_INFO_DEV(frame_relay_node.get(), __func__, false, "InitConfig JSON: {}", init_config_json);
    }
    runtime_config->parse_from_node_parameters(runtime_config.get(), frame_relay_node.get());
    {
        RDX_INFO_DEV(frame_relay_node.get(), __func__, false, "{}", "Converting runtime config to JSON");
        auto runtime_config_json = JS::serializeStruct(*runtime_config);
        RDX_INFO_DEV(frame_relay_node.get(), __func__, false, "RuntimeConfig JSON: {}", runtime_config_json);
    }

    RDX_INFO_DEV(frame_relay_node.get(), __func__, false, "{}", "Initializing node");
    frame_relay_node->init(init_config, runtime_config);

    //! Start the node
    frame_relay_node->start();

    //! Keep the node running
    rclcpp::spin(frame_relay_node);

    //! Stop the node before shutdown
    frame_relay_node->stop();

    //! Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
