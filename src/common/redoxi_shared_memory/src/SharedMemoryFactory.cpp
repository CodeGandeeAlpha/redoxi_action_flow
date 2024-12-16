#include <redoxi_shared_memory/SharedMemoryFactory.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>
#include <cstdlib>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

static const char *ShmBaseClassName = "redoxi_works::shared_memory::SharedMemoryClient";
static const char *ShmBasePackageName = "redoxi_shared_memory";
static const char *VineyardClassName = "redoxi_works::shared_memory::VineyardShmClient";


namespace redoxi_works::shared_memory
{

using ClassLoaderType = pluginlib::ClassLoader<SharedMemoryClient>;

struct SharedMemoryFactory::Impl {
    std::shared_ptr<ClassLoaderType> loader;
    ~Impl()
    {
        RDX_INFO_DEV(nullptr, __func__, "{}", "destroying impl");
        loader = nullptr;
    }
};

SharedMemoryFactory &SharedMemoryFactory::get_instance()
{
    static SharedMemoryFactory instance;
    if (!instance.m_impl) {
        instance.m_impl = std::make_shared<Impl>();
        instance.m_impl->loader = std::make_shared<ClassLoaderType>(ShmBasePackageName, ShmBaseClassName);

        // IMPORTANT: clean up all clients, because loader will be destroyed when ros shutdown, NOT when main function exits
        // if clients persist after ros shutdown, they will cause segmentation fault (and ros warning)
        rclcpp::on_shutdown([&]() {
            // clean up all clients, because loader is about to be destroyed
            instance.m_clients_by_config.clear();
        });
    }
    return instance;
}

SharedMemoryConfig SharedMemoryFactory::get_default_shm_config(const rclcpp::Node *node)
{
    if (node) {
        return get_shm_config_from_node(node);
    } else {
        return get_shm_config_from_env();
    }
}

int SharedMemoryFactory::destroy_client(std::shared_ptr<SharedMemoryClient> client)
{
    if (!client) {
        return -1;
    }

    // look for the client in the cache
    SharedMemoryConfig config;
    config.region_key = client->get_region_key();
    config.service_type = client->get_service_type();
    auto it = m_clients_by_config.find(config);
    if (it != m_clients_by_config.end()) {
        RDX_INFO_DEV(nullptr, __func__, "removing client of service type {} and region key {} in cache",
                     config.service_type, config.region_key);
        m_clients_by_config.erase(it);
        return 0;
    }

    RDX_INFO_DEV(nullptr, __func__, "client of service type {} and region key {} not found in cache, skipping",
                 config.service_type, config.region_key);
    return -1;
}

std::shared_ptr<SharedMemoryClient> SharedMemoryFactory::get_default_client(rclcpp::Node *node)
{
    SharedMemoryConfig config = get_default_shm_config(node);
    if (!config.is_valid()) {
        RDX_WARN_DEV(nullptr, __func__, "invalid config, service type {} and region key {}, skipping",
                     config.service_type, config.region_key);
        return nullptr;
    }
    return get_client(config);
}

std::shared_ptr<SharedMemoryClient> SharedMemoryFactory::get_client(const SharedMemoryConfig &config)
{
    if (!config.is_valid()) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "Invalid config, cannot create shm client");
        return nullptr;
    }

    // do we already have a client of this config?
    RDX_INFO_DEV(nullptr, __func__, "looking for client of service type {} and region key {}",
                 config.service_type, config.region_key);
    auto it = m_clients_by_config.find(config);
    if (it != m_clients_by_config.end()) {
        RDX_INFO_DEV(nullptr, __func__, "found client of service type {} and region key {} in cache",
                     config.service_type, config.region_key);
        // yes, just return it
        return it->second;
    }

    // create a new client
    RDX_INFO_DEV(nullptr, __func__, "client of service type {} and region key {} not found, creating a new one",
                 config.service_type, config.region_key);
    auto client = _create_client_by_config(config);
    if (!client) {
        RDX_INFO_DEV(nullptr, __func__, "{}", "failed to create shm client");
        return nullptr;
    }

    // add to cache
    m_clients_by_config[config] = client;
    RDX_INFO_DEV(nullptr, __func__, "added client of service type {} and region key {} to cache",
                 config.service_type, config.region_key);
    return client;
}

std::shared_ptr<SharedMemoryClient> SharedMemoryFactory::_create_client_by_config(
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

    auto client = _create_client_by_service_type(service_type);
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

std::shared_ptr<SharedMemoryClient> SharedMemoryFactory::_create_client_by_service_type(
    const std::string &service_type)
{
    // hold this loader permanently, you should not destroy loader before the program exits
    // static auto loader = pluginlib::ClassLoader<SharedMemoryClient>(ShmBasePackageName, ShmBaseClassName);
    auto &loader = *get_instance().m_impl->loader;
    if (service_type == config_values::service_types::Vineyard) {
        auto client = loader.createSharedInstance(VineyardClassName);
        return client;
    }

    return nullptr;
}

std::string SharedMemoryFactory::_get_shm_service_name_from_env()
{
    auto service_type = std::getenv(config_keys::env::ServiceType.data());
    if (service_type) {
        return service_type;
    }

    return "";
}

std::string SharedMemoryFactory::_get_shm_region_key_from_env()
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

SharedMemoryConfig SharedMemoryFactory::get_shm_config_from_env()
{
    SharedMemoryConfig config;
    config.service_type = _get_shm_service_name_from_env();
    config.region_key = _get_shm_region_key_from_env();
    return config;
}

SharedMemoryFactory::~SharedMemoryFactory()
{
    // destroy all clients
    RDX_INFO_DEV(nullptr, __func__, "{}", "destroying all clients");
    m_clients_by_config.clear();
    // RDX_INFO_DEV(nullptr, __func__, "unloading library for class {}", ShmBaseClassName);
    // m_impl->loader->unloadLibraryForClass(ShmBaseClassName);
    // m_impl = nullptr;
}

} // namespace redoxi_works::shared_memory
