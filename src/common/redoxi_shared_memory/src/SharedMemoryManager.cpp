#include <redoxi_shared_memory/SharedMemoryManager.hpp>
#include <redoxi_common_cpp/ros_utils/common.hpp>

namespace redoxi_works::shared_memory
{

SharedMemoryManager &SharedMemoryManager::get_instance()
{
    static SharedMemoryManager instance;
    return instance;
}

std::shared_ptr<SharedMemoryClient> SharedMemoryManager::get_client(const SharedMemoryConfig &config)
{
    return get_instance()._get_client(config);
}

std::shared_ptr<SharedMemoryClient> SharedMemoryManager::_get_client(const SharedMemoryConfig &config)
{
    if (!config.is_valid()) {
        RDX_WARN_DEV(nullptr, __func__, "No shm client for invalid config: service_type={}, region_key={}",
                     config.service_type, config.region_key);
        return nullptr;
    }

    //! Use config as unique identifier for clients
    RDX_INFO_DEV(nullptr, __func__, "Looking for client with config: service_type={}, region_key={}",
                 config.service_type, config.region_key);
    auto it = m_clients_by_config.find(config);
    if (it != m_clients_by_config.end()) {
        RDX_INFO_DEV(nullptr, __func__, "Found client with config: service_type={}, region_key={}",
                     config.service_type, config.region_key);
        return it->second;
    }

    //! Create new client if not found
    RDX_INFO_DEV(nullptr, __func__, "Creating new client with config: service_type={}, region_key={}",
                 config.service_type, config.region_key);
    auto client = SharedMemoryFactory::create_client_by_config(config);
    if (client) {
        RDX_INFO_DEV(nullptr, __func__, "Created new client with config: service_type={}, region_key={}",
                     config.service_type, config.region_key);
        m_clients_by_config[config] = client;
    }
    return client;
}

std::shared_ptr<SharedMemoryClient> SharedMemoryManager::get_default_client(rclcpp::Node *node)
{
    auto config = node ? SharedMemoryFactory::get_shm_config_from_node(node)
                       : SharedMemoryFactory::get_shm_config_from_env();
    auto client = get_client(config);
    if (client) {
        RDX_INFO_DEV(nullptr, __func__, "Got shm client from {} with config: service_type={}, region_key={}",
                     node ? "node" : "env", config.service_type, config.region_key);
        return client;
    } else {
        RDX_WARN_DEV(nullptr, __func__, "{}", node ? "Failed to create default shm client from node" : "Invalid default shared memory config from env");
        return nullptr;
    }
}

} // namespace redoxi_works::shared_memory