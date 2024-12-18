#include <psg_pose_detector/Driver.hpp>

namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    //! Create a FrameRelayNode with a specific name and options
    auto psg_pose_detector_driver = std::make_shared<rdx::PSGPoseDetectorDriver>("psg_pose_detector_driver");

    //! Initialize the node with default configuration
    auto init_config = std::make_shared<rdx::PSGPoseDetectorDriver::InitConfig_t>();
    init_config->parse_from_node_parameters(init_config.get(), psg_pose_detector_driver.get());
    {
        RDX_INFO_DEV(psg_pose_detector_driver.get(), __func__, false, "{}", "Converting init config to JSON");
        auto init_config_json = JS::serializeStruct(*init_config);
        RDX_INFO_DEV(psg_pose_detector_driver.get(), __func__, false, "InitConfig JSON: {}", init_config_json);
    }
    auto runtime_config = std::make_shared<rdx::PSGPoseDetectorDriver::RuntimeConfig_t>();
    runtime_config->parse_from_node_parameters(runtime_config.get(), psg_pose_detector_driver.get());
    {
        RDX_INFO_DEV(psg_pose_detector_driver.get(), __func__, false, "{}", "Converting runtime config to JSON");
        auto runtime_config_json = JS::serializeStruct(*runtime_config);
        RDX_INFO_DEV(psg_pose_detector_driver.get(), __func__, false, "RuntimeConfig JSON: {}", runtime_config_json);
    }

    psg_pose_detector_driver->init(init_config, runtime_config);

    //! Start the node
    psg_pose_detector_driver->open();
    psg_pose_detector_driver->start();

    //! Keep the node running
    rclcpp::spin(psg_pose_detector_driver);

    //! Stop the node before shutdown
    psg_pose_detector_driver->stop();
    psg_pose_detector_driver->close();
    //! Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
