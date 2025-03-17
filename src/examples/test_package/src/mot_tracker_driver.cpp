#include <test_package/_pch.hpp>

#include <universal_mot_trackers/TrackerDriverNode.hpp>
#include <spdlog/spdlog.h>
#include <filesystem>

namespace rdx = redoxi_works;
namespace rdx_models = redoxi_works::model_nodes;
using TrackerDriverNode_t = rdx_models::universal_mot_trackers::TrackerDriverNode;

void print_config_jsons()
{
    using InitConfig_t = TrackerDriverNode_t::InitConfig_t;
    using RuntimeConfig_t = TrackerDriverNode_t::RuntimeConfig_t;
    rdx::NodeConfigTemplate<InitConfig_t, RuntimeConfig_t> node_config;

    {
        TrackerDriverNode_t::InitConfig_t init_config;
        init_config.input_port_config->set_action_name("in/tracking_request");
        node_config.init_config = init_config;
    }

    {
        TrackerDriverNode_t::RuntimeConfig_t runtime_config;
        runtime_config.enable_blocking_mode = true;
        node_config.runtime_config = runtime_config;
    }

    std::string json_str = JS::serializeStruct(node_config);
    std::cout << "Node config: " << std::endl;
    std::cout << json_str << std::endl;
}

int main(int argc, char **argv)
{
    //! Initialize ROS2
    spdlog::info("Initializing ROS2...");
    rclcpp::init(argc, argv);

    //! Create tracker driver node
    spdlog::info("Creating tracker driver node...");
    auto node = std::make_shared<TrackerDriverNode_t>("test_tracker_driver_node");

    //! Create and parse init config
    spdlog::info("Creating and parsing init config...");
    auto init_config = std::make_shared<TrackerDriverNode_t::InitConfig_t>();
    init_config->parse_from_node_parameters(init_config.get(), node.get());

    //! Create and parse runtime config
    spdlog::info("Creating and parsing runtime config...");
    auto runtime_config = std::make_shared<TrackerDriverNode_t::RuntimeConfig_t>();
    runtime_config->parse_from_node_parameters(runtime_config.get(), node.get());

    //! Initialize node
    spdlog::info("Initializing node...");
    node->init(init_config, runtime_config);

    //! Start node
    spdlog::info("Starting node...");
    node->open();
    node->start();

    //! Spin node
    spdlog::info("Spinning node...");
    rclcpp::spin(node->get_node_base_interface());

    //! Stop node
    spdlog::info("Stopping node...");
    node->stop();
    node->close();

    //! Cleanup
    spdlog::info("Shutting down ROS2...");
    rclcpp::shutdown();
    return 0;
}
