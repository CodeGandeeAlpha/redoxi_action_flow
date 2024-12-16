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
    //! Use config as unique identifier for clients
    auto it = m_clients_by_config.find(config);
    if (it != m_clients_by_config.end()) {
        return it->second;
    }

    //! Create new client if not found
    auto client = SharedMemoryFactory::create_client_by_config(config);
    if (client) {
        m_clients_by_config[config] = client;
    }
    return client;
}

} // namespace redoxi_works::shared_memory