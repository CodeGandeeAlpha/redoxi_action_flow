#include <redoxi_samples_nodes/sinks/DetectionRelayNode.hpp>
#include <json_struct/json_struct.h>
#include <spdlog/spdlog.h>

namespace rdx = redoxi_works;
namespace rdx_nodes = redoxi_works::common_nodes;

using InitConfig_t = rdx::DetectionRelayNode::InitConfig_t;
using RuntimeConfig_t = rdx::DetectionRelayNode::RuntimeConfig_t;
using CombinedConfig_t = rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t>;

void print_json()
{
    CombinedConfig_t combined_config;
    auto combined_json = JS::serializeStruct(combined_config);
    std::cout << combined_json << std::endl;
}

int main(int argc, char **argv)
{
    // print_json();
    // return 0;
    spdlog::info("Initializing ROS...");
    rclcpp::init(argc, argv);

    spdlog::info("Creating DetectionRelayNode...");
    auto node = std::make_shared<rdx::DetectionRelayNode>("detection_relay_node");

    spdlog::info("Setting up initialization configuration...");
    auto init_config = std::make_shared<rdx::DetectionRelayNode::InitConfig_t>();
    // init_config->input_port_config->set_action_name("in/detection");
    // init_config->publish_detection_topic = "out/relayed_detection";
    // init_config->publish_visualization_topic = "out/relayed_visualization";
    init_config->parse_from_node_parameters(init_config.get(), node.get());

    spdlog::info("Setting up runtime configuration...");
    auto runtime_config = std::make_shared<rdx::DetectionRelayNode::RuntimeConfig_t>();
    // runtime_config->enable_blocking_mode = false;
    // runtime_config->enable_visualization = false;
    runtime_config->parse_from_node_parameters(runtime_config.get(), node.get());

    spdlog::info("Initializing node with configurations...");
    node->init(init_config, runtime_config);

    spdlog::info("Starting the node...");
    node->start();

    spdlog::info("Node started successfully. Spinning until shutdown...");
    rclcpp::spin(node);

    spdlog::info("Stopping the node...");
    node->stop();

    spdlog::info("Shutting down ROS...");
    rclcpp::shutdown();
    return 0;
}
