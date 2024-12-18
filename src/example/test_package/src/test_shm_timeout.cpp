#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redoxi_shared_memory/SharedMemoryClient.hpp>
#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iostream>

namespace rdx_shm = redoxi_works::shared_memory;
const std::string vineyard_socket = "/soft/data/vineyard.sock";
namespace rdx = redoxi_works;

int main(int argc, char **argv)
{
    RDX_INFO_DEV(nullptr, __func__, "{}", "Starting test_v6d_plugin");
    // do this so that we have rclcpp loggers
    rclcpp::init(argc, argv);

    // pluginlib::ClassLoader<rdx_shm::SharedMemoryClient> shm_loader("redoxi_shared_memory",
    //                                                                "redoxi_works::shared_memory::SharedMemoryClient");
    rdx_shm::SharedMemoryConfig config;
    config = rdx_shm::SharedMemoryFactory::get_instance().get_shm_config_from_env();
    RDX_INFO_DEV(nullptr, __func__, "shm config, service type: {}, region key: {}",
                 config.service_type, config.region_key);

    try {
        // auto client = shm_loader.createSharedInstance("redoxi_works::shared_memory::VineyardShmClient");
        // auto client = rdx_shm::SharedMemoryFactory::create_client_by_config(config);
        auto client = rdx_shm::SharedMemoryFactory::get_instance().get_default_client().lock();
        if (!client) {
            spdlog::error("Failed to create shared memory client");
            return -1;
        }
        spdlog::info("Client created");

        // set default alive duration to 1 seconds
        client->set_default_alive_duration(std::chrono::seconds(1));
        client->set_max_alive_duration(std::chrono::seconds(3));

        // destroy the client
        // spdlog::info("Destroying client");
        // rdx_shm::SharedMemoryFactory::get_instance().destroy_client(client);
        // spdlog::info("Client destroyed");

        // connect to vineyard
        if (!client->is_connected()) {
            auto ret = client->connect(config);
            if (ret == 0)
                spdlog::info("Client connected to vineyard");
            else
                spdlog::error("Client failed to connect to vineyard");
        }

        // put a cv::Mat into vineyard
        cv::Mat mat(3, 4, CV_8UC1);
        cv::randu(mat, cv::Scalar(0), cv::Scalar(255));

        rdx_shm::ShmPutOptions put_options;
        // put_options.alive_duration = std::chrono::seconds(0.5);
        put_options.on_expired = [](
                                     const rdx_shm::ObjectIdentifier &oid,
                                     rdx_shm::SharedMemoryClient *client,
                                     const auto &time_now,
                                     const rdx_shm::ShmPutOptions &opt) {
            (void)client;
            spdlog::info("Object expired, object id = {}, options = {}, time now = {}",
                         oid.to_string(), opt.to_string(), time_now.time_since_epoch().count());
            return rdx_shm::ShmPutOptions::ExpiredAction::DontCare;
        };

        // print it here
        spdlog::info("Mat put into vineyard:");
        spdlog::info("{}", (std::ostringstream() << mat).str());

        // put it into vineyard
        rdx_shm::ObjectIdentifier oid;
        {
            auto datablock = client->create_datablock();
            datablock->from_cvmat(mat);
            auto ret = client->put_data(&oid, datablock.get(), nullptr, put_options);
            if (ret == 0)
                spdlog::info("Put to vineyard, Object ID: {}, options = {}", oid.id.value(), put_options.to_string());
            else
                spdlog::error("Put to vineyard failed");
        }

        // get it from vineyard
        auto datablock = client->create_datablock();
        {
            auto ret = client->get_data(datablock.get(), nullptr, oid);
            if (ret == 0)
                spdlog::info("Get from vineyard, Object ID: {}", oid.id.value());
            else
                spdlog::error("Get from vineyard failed");

            // print it here
            cv::Mat mat_get;
            datablock->get_as_cvmat(&mat_get);
            spdlog::info("Mat get from vineyard:");
            spdlog::info("{}", (std::ostringstream() << mat_get).str());
        }

        // wait for the object to expire
        spdlog::info("Waiting for the object to expire");
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // get it again
        {
            auto datablock = client->create_datablock();
            auto ret = client->get_data(datablock.get(), nullptr, oid);
            if (ret == 0)
                spdlog::info("Get from vineyard, Object ID: {}", oid.id.value());
            else
                spdlog::error("Get from vineyard failed");
        }

        // remove it from vineyard
        {
            spdlog::info("Before delete,press Enter to continue");
            std::cin.get();
            auto ret = client->delete_object(oid);
            if (ret == 0) {
                spdlog::info("Remove from vineyard, Object ID: {}", oid.id.value());
            } else {
                spdlog::error("Remove from vineyard failed");
            }
        }
    } catch (const pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("test_v6d_plugin"), "The plugin failed to load for some reason. Error: %s", ex.what());
    }

    spdlog::info("Shutting down");
    rclcpp::shutdown();
    return 0;
}