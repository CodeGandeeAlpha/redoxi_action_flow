#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <cstdlib>
#include <pluginlib/class_loader.hpp>
#include <rcutils/logging_macros.h>

static const char *ShmBaseClassName = "redoxi_works::shared_memory::SharedMemoryClient";
static const char *ShmBasePackageName = "redoxi_shared_memory";
static const char *VineyardClassName = "redoxi_works::shared_memory::VineyardShmClient";


namespace redoxi_works
{
namespace shared_memory
{

std::shared_ptr<SharedMemoryClient> SharedMemoryFactory::create_client_by_type(const std::string &service_type)
{
    // hold this loader permanently, you should not destroy loader before the program exits
    static auto loader = pluginlib::ClassLoader<SharedMemoryClient>(ShmBasePackageName, ShmBaseClassName);

    if (service_type == shm_service_types::Vineyard) {
        auto client = loader.createSharedInstance(VineyardClassName);
        return client;
    }

    return nullptr;
}

std::shared_ptr<SharedMemoryClient> SharedMemoryFactory::create_client_by_env()
{
    auto service_type = get_shm_service_type_from_env();
    auto region_key = get_shm_region_key_from_env();

    if (service_type.empty()) {
        RCUTILS_LOG_ERROR("service_type is empty");
        return nullptr;
    }

    if (region_key.empty()) {
        RCUTILS_LOG_ERROR("region_key is empty");
        return nullptr;
    }

    RCUTILS_LOG_INFO("shared memory service found");
    RCUTILS_LOG_INFO("service_type: %s, region_key: %s", service_type.c_str(), region_key.c_str());

    try {
        if (service_type == shm_service_types::Vineyard) {
            RCUTILS_LOG_INFO("creating vineyard shared memory client");
            static auto loader = pluginlib::ClassLoader<SharedMemoryClient>(ShmBasePackageName, ShmBaseClassName);
            auto client = loader.createSharedInstance(VineyardClassName);
            if (client->connect(region_key) == 0) {
                RCUTILS_LOG_INFO("vineyard shared memory client created and connected");
                return client;
            }
        }
    } catch (const pluginlib::PluginlibException &ex) {
        RCUTILS_LOG_ERROR("The plugin failed to load, error: %s", ex.what());
    }

    return nullptr;
}

std::string SharedMemoryFactory::get_shm_service_type_from_env()
{
    auto service_type = std::getenv(env_names::ShmServiceType);
    if (service_type) {
        return service_type;
    }

    return "";
}

std::string SharedMemoryFactory::get_shm_region_key_from_env()
{
    auto region_key = std::getenv(env_names::RegionKey);
    if (region_key) {
        return region_key;
    }

    return "";
}

} // namespace shared_memory
} // namespace redoxi_works
