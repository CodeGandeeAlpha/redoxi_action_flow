#include <redoxi_video_reader/generators/SimpleActionGenerator.hpp>

namespace rdx = redoxi_works;
using Node_t = rdx::SimpleActionGenerator;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node_t>("simple_action_generator");
    auto init_config = std::make_shared<Node_t::InitConfig_t>();
    auto runtime_config = std::make_shared<Node_t::RuntimeConfig_t>();

    init_config->from_parameters(node.get());
    runtime_config->from_parameters(node.get());

    node->init(init_config, runtime_config);
    node->open();
    node->start();

    rclcpp::spin(node);

    node->close();
    rclcpp::shutdown();
    return 0;
}