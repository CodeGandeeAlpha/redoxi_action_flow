#include <video_reader_orbbec/VideoReaderOrbbec.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //! Create a shared pointer for the VideoReaderOrbbec node
    auto video_generator = std::make_shared<rdx::VideoReaderOrbbec>("orbbec_video_source");

    //! Create and initialize the node's configuration
    auto init_config = std::make_shared<rdx::VideoReaderOrbbec::InitConfig_t>();
    auto runtime_config = std::make_shared<rdx::VideoReaderOrbbec::RuntimeConfig_t>();

    init_config->parse_from_node_parameters(init_config.get(), video_generator.get());
    {
        RDX_INFO_DEV(video_generator.get(), __func__, false, "{}", "Converting init config to JSON");
        auto init_config_json = JS::serializeStruct(*init_config);
        RDX_INFO_DEV(video_generator.get(), __func__, false, "InitConfig JSON: {}", init_config_json);
    }
    runtime_config->parse_from_node_parameters(runtime_config.get(), video_generator.get());
    {
        RDX_INFO_DEV(video_generator.get(), __func__, false, "{}", "Converting runtime config to JSON");
        auto runtime_config_json = JS::serializeStruct(*runtime_config);
        RDX_INFO_DEV(video_generator.get(), __func__, false, "RuntimeConfig JSON: {}", runtime_config_json);
    }

    //! Initialize the node with the configuration
    video_generator->init(init_config, runtime_config);

    //! Open the node to prepare it for operation
    video_generator->open();

    //! Start the node to begin generating random frames
    video_generator->start();

    //! Spin the node to process callbacks and keep it running
    rclcpp::spin(video_generator->get_node_base_interface());

    //! Stop and close the node before shutting down
    video_generator->stop();
    video_generator->close();

    //! Shutdown ROS 2
    rclcpp::shutdown();
}