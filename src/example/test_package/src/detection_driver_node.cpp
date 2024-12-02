#include <redoxi_samples_nodes/drivers/DetectionRequestDriver.hpp>
#include <json_struct/json_struct.h>
#include <spdlog/spdlog.h>
#include <redoxi_public_msgs/action/process_detections.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

namespace rdx = redoxi_works;
namespace rdx_nodes = redoxi_works::common_nodes;

using NodeType = redoxi_works::samples_nodes::drivers::DetectionRequestDriver;
using NodeInitConfig = NodeType::InitConfig_t;
using NodeRuntimeConfig = NodeType::RuntimeConfig_t;
using NodeConfigTemplate = rdx::NodeConfigTemplate<NodeInitConfig, NodeRuntimeConfig>;
using DetectionRequestAction = redoxi_public_msgs::action::ProcessDetections;

void print_json()
{
    NodeConfigTemplate config;
    auto json_string = JS::serializeStruct(config);
    std::cout << json_string << std::endl;

    // std::cout << rosidl_generator_traits::name<DetectionRequestAction::Goal>() << std::endl;
}

int main(int argc, char **argv)
{
    // print_json();
    // return 0;

    //! Initialize ROS
    rclcpp::init(argc, argv);

    //! Create node instance
    auto node = std::make_shared<NodeType>("detection_driver_node");

    //! Setup initialization configuration
    auto init_config = std::make_shared<NodeInitConfig>();
    init_config->parse_from_node_parameters(init_config.get(), node.get());

    //! Setup runtime configuration
    auto runtime_config = std::make_shared<NodeRuntimeConfig>();
    runtime_config->parse_from_node_parameters(runtime_config.get(), node.get());

    //! Initialize node with configurations
    node->init(init_config, runtime_config);

    //! Start the node
    node->start();

    //! Spin until shutdown
    rclcpp::spin(node);

    //! Stop node and shutdown
    node->stop();
    rclcpp::shutdown();
    return 0;
}