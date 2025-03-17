#include <test_package/_pch.hpp>

#include <redoxi_common_nodes/async_action_port/AsyncActionInputPort.hpp>
#include <redoxi_samples_nodes/sinks/FrameRelayNode.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <spdlog/spdlog.h>

namespace rdx = redoxi_works;
using RosNode_t = rdx::FrameRelayNode;

void print_init_config_json()
{
    using InitConfig_t = RosNode_t::InitConfig_t;
    using RuntimeConfig_t = RosNode_t::RuntimeConfig_t;
    rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t> node_config;

    auto init_config = std::make_shared<rdx::FrameRelayNodeInitConfig>();
    init_config->input_port_config->set_action_name("in/frame");
    init_config->publish_topic = "out/relayed_frame";
    init_config->debug_topic_frame_accepted = "debug/frame_accepted";
    init_config->debug_topic_frame_rejected = "debug/frame_rejected";

    auto runtime_config = std::make_shared<rdx::FrameRelayNodeRuntimeConfig>();
    node_config.init_config = *init_config;
    node_config.runtime_config = *runtime_config;
    auto js_string = JS::serializeStruct(node_config);
    std::cout << js_string << std::endl;
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
    spdlog::info("Creating init config");
    auto init_config = std::make_shared<rdx::FrameRelayNodeInitConfig>();

    spdlog::info("Creating runtime config");
    auto runtime_config = std::make_shared<rdx::FrameRelayNodeRuntimeConfig>();

    spdlog::info("Parsing init config from node parameters");
    init_config->parse_from_node_parameters(init_config.get(), frame_relay_node.get());
    runtime_config->parse_from_node_parameters(runtime_config.get(), frame_relay_node.get());

    spdlog::info("Initializing node");
    frame_relay_node->init(init_config, runtime_config);

    //! Start the node
    spdlog::info("Starting node");
    frame_relay_node->start();

    //! Keep the node running
    spdlog::info("Spinning node");
    rclcpp::spin(frame_relay_node->get_node_base_interface());

    //! Stop the node before shutdown
    spdlog::info("Stopping node");
    frame_relay_node->stop();

    //! Shutdown ROS
    spdlog::info("Shutting down ROS");
    rclcpp::shutdown();
    return 0;
}
