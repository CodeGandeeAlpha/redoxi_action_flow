#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <cstdlib>
#include <pluginlib/class_loader.hpp>
#include <rcutils/logging_macros.h>

static const char *ShmBaseClassName = "redoxi_works::shared_memory::SharedMemoryClient";
static const char *ShmBasePackageName = "redoxi_shared_memory";
static const char *VineyardClassName = "redoxi_works::shared_memory::VineyardShmClient";

namespace redoxi_works::shared_memory
{

std::shared_ptr<SharedMemoryClient> SharedMemoryFactory::create_client_by_config(
    const SharedMemoryConfig &config)
{
    std::string service_type = config.service_type;
    std::string region_key = config.region_key;

    // now, both should be set
    if (service_type.empty() || region_key.empty()) {
        RDX_INFO_DEV(nullptr, __func__, "{}",
                     "Service type or region key is not set, cannot create shm client");
        return nullptr;
    }

    auto client = create_client_by_service_type(service_type);
    if (!client) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "Failed to create shm client");
        return nullptr;
    }

    // connect to shm
    auto ret = client->connect(region_key);
    if (ret != 0) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "Failed to connect to shm");
        return nullptr;
    }

    return client;
}

std::shared_ptr<SharedMemoryClient> SharedMemoryFactory::create_client_by_service_type(
    const std::string &service_type)
{
    // hold this loader permanently, you should not destroy loader before the program exits
    static auto loader = pluginlib::ClassLoader<SharedMemoryClient>(ShmBasePackageName, ShmBaseClassName);

    if (service_type == config_values::service_types::Vineyard) {
        auto client = loader.createSharedInstance(VineyardClassName);
        return client;
    }

    return nullptr;
}

std::string SharedMemoryFactory::get_shm_service_name_from_env()
{
    auto service_type = std::getenv(config_keys::env::ServiceType.data());
    if (service_type) {
        return service_type;
    }

    return "";
}

std::string SharedMemoryFactory::get_shm_region_key_from_env()
{
    auto region_key = std::getenv(config_keys::env::RegionKey.data());
    if (region_key) {
        return region_key;
    }

    return "";
}

SharedMemoryConfig SharedMemoryFactory::get_shm_config_from_node(const rclcpp::Node *node)
{
    // currently, just read from env
    SharedMemoryConfig config;
    node->get_parameter(config_keys::node::ServiceType.data(), config.service_type);
    node->get_parameter(config_keys::node::RegionKey.data(), config.region_key);

    return config;
}
//! Get the shared memory config from env variables
SharedMemoryConfig SharedMemoryFactory::get_shm_config_from_env()
{
    SharedMemoryConfig config;
    config.service_type = get_shm_service_name_from_env();
    config.region_key = get_shm_region_key_from_env();
    return config;
}

} // namespace redoxi_works::shared_memory
