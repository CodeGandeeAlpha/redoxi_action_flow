#include <redoxi_shared_memory/SharedMemoryManager.hpp>

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
    //! Use region key as unique identifier for clients
    std::string key = config.region_key;
    if (key.empty() && config.read_setting_from_env) {
        key = SharedMemoryFactory::get_shm_region_key_from_env();
    }

    auto it = m_clients.find(key);
    if (it != m_clients.end()) {
        return it->second;
    }

    //! Create new client if not found
    auto client = SharedMemoryFactory::create_client_by_config(config);
    if (client) {
        m_clients[key] = client;
    }
    return client;
}

} // namespace redoxi_works::shared_memory